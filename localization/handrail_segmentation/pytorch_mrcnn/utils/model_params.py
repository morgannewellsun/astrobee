
class Model_Params():
    def __init__(self):
        self.hyperparams = {"learning_rate": 0.0001,
                            "momentum": 0.9,
                            "weight_decay": 0.0005,
                            "burn_in": 1000,
                            "lr_steps": 4,
                            "batch_size": 2,
                            "num_workers": 4}
        self.optimizer = {"step_size": 3,
                                "gamma": 0.1}

        self.checkpoint_interval = 10
