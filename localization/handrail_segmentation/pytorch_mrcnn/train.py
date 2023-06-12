import sys
import math

import torch
import torchvision
import tqdm
import utils.transforms as T
import utils.utils as utils
from torchvision.models.detection.faster_rcnn import FastRCNNPredictor
from torchvision.models.detection.mask_rcnn import MaskRCNNPredictor
from utils.dataset import AstrobeeHandrailDataset
from utils.engine import evaluate, train_one_epoch
from utils.model_params import Model_Params


def get_transform(train):
    transforms = []
    if train:
        transforms.append(T.RandAugment())
    transforms.append(T.ToTensor())
    return T.Compose(transforms)


def get_model_instance_segmentation(num_classes):
    # load an instance segmentation model pre-trained on COCO
    model = torchvision.models.detection.maskrcnn_resnet50_fpn(pretrained=True)

    # get number of input features for the classifier
    in_features = model.roi_heads.box_predictor.cls_score.in_features
    # replace the pre-trained head with a new one
    model.roi_heads.box_predictor = FastRCNNPredictor(in_features, num_classes)

    # now get the number of input features for the mask classifier
    in_features_mask = model.roi_heads.mask_predictor.conv5_mask.in_channels
    hidden_layer = 256
    # and replace the mask predictor with a new one
    model.roi_heads.mask_predictor = MaskRCNNPredictor(
        in_features_mask, hidden_layer, num_classes
    )
    # model.load_state_dict(torch.load('/home/anaveen/Documents/nasa_ws/astrobee-detection-pipeline/src/handrail_segmentation/src/weights/mrcnn_ckpt_60.pth'))
    return model


def main():

    params = Model_Params()
    # train on the GPU or on the CPU, if a GPU is not available
    device = torch.device("cuda") if torch.cuda.is_available() else torch.device("cpu")

    # our dataset has five classes only - background and handrail 8.5, handrail 21.5, handrail 30, handrail 41.5
    num_classes = 5
    # use our dataset and defined transformations
    dataset = AstrobeeHandrailDataset("data", get_transform(train=True))
    dataset_test = AstrobeeHandrailDataset("data", get_transform(train=False))

    # split the dataset in train and test set
    indices = torch.randperm(len(dataset)).tolist()
    cutoff = int(len(dataset) * 0.8)
    dataset = torch.utils.data.Subset(dataset, indices[:cutoff])
    dataset_test = torch.utils.data.Subset(dataset_test, indices[cutoff:])

    # define training and validation data loaders
    data_loader = torch.utils.data.DataLoader(
        dataset,
        batch_size=params.hyperparams["batch_size"],
        shuffle=True,
        num_workers=params.hyperparams["num_workers"],
        collate_fn=utils.collate_fn,
    )

    data_loader_test = torch.utils.data.DataLoader(
        dataset_test,
        batch_size=2,
        shuffle=False,
        num_workers=params.hyperparams["num_workers"],
        collate_fn=utils.collate_fn,
    )

    # get the model using our helper function
    model = get_model_instance_segmentation(num_classes)

    # move model to the right device
    model.to(device)

    # construct an optimizer
    optimizer_params = [p for p in model.parameters() if p.requires_grad]
    optimizer = torch.optim.SGD(
        optimizer_params,
        lr=params.hyperparams["learning_rate"],
        momentum=params.hyperparams["momentum"],
        weight_decay=params.hyperparams["weight_decay"],
    )
    # and a learning rate scheduler
    lr_scheduler = torch.optim.lr_scheduler.StepLR(
        optimizer,
        step_size=params.optimizer["step_size"],
        gamma=params.optimizer["gamma"],
    )

    # let's train it for 10 epochs
    num_epochs = 200

    for epoch in range(num_epochs):
        # # train for one epoch, printing every 10 iterations
        # train_one_epoch(model, optimizer, data_loader, device, epoch, print_freq=10)
        # # update the learning rate
        # lr_scheduler.step()
        # # evaluate on the test dataset
        # evaluate(model, data_loader_test, device=device)

        print("\n---- Training Model ----")
        model.train()
        lr_scheduler = None
        if epoch == 0:
            warmup_factor = 1.0 / 1000
            warmup_iters = min(1000, len(data_loader) - 1)

            lr_scheduler = utils.warmup_lr_scheduler(
                optimizer, warmup_iters, warmup_factor
            )

        for batch_i, (images, targets) in enumerate(
            tqdm.tqdm(data_loader, desc=f"Training Epoch {epoch}")
        ):
            images = list(image.to(device) for image in images)
            targets = [{k: v.to(device) for k, v in t.items()} for t in targets]

            loss_dict = model(images, targets)

            losses = sum(loss for loss in loss_dict.values())

            # reduce losses over all GPUs for logging purposes
            loss_dict_reduced = utils.reduce_dict(loss_dict)
            losses_reduced = sum(loss for loss in loss_dict_reduced.values())

            loss_value = losses_reduced.item()

            if not math.isfinite(loss_value):
                print("Loss is {}, stopping training".format(loss_value))
                print(loss_dict_reduced)
                sys.exit(1)

            optimizer.zero_grad()
            losses.backward()
            optimizer.step()

            if lr_scheduler is not None:
                lr_scheduler.step()

        if epoch % params.checkpoint_interval == 0 or epoch == num_epochs - 1:
            checkpoint_path = f"checkpoints/handrail_{epoch}.pth"
            print(f"---- Saving checkpoint to: '{checkpoint_path}' ----")
            torch.save(model.state_dict(), checkpoint_path)
            evaluate(model, data_loader_test, device=device)

    print("That's it!")


if __name__ == "__main__":
    main()
