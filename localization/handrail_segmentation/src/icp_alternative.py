import numpy as np
from sklearn.decomposition import PCA


def construct_handle_frame_transform(handle_pointcloud: np.ndarray, surface_pointcloud: np.ndarray) -> np.ndarray:
    """
    Constructs a handle-centered frame of reference and associated homogeneous transform matrix.
    :param handle_pointcloud: np.ndarray of shape (n_points, 3), representing points in an input frame.
    :param surface_pointcloud: np.ndarray of shape (n_points, 3), representing points in an input frame.
    :return:
    np.ndarray of shape (4, 4) representing the transform from the handle frame to the input frame.
    """

    # Fit geometry
    handle_direction_vector, handle_pointcloud_mean = fit_line_to_pointcloud(handle_pointcloud)
    surface_normal_vector, surface_pointcloud_mean = fit_plane_to_pointcloud(surface_pointcloud)

    # Ensure handle direction is parallel to surface plane
    handle_direction_vector -= vector_projection(handle_direction_vector - surface_normal_vector)
    handle_direction_vector /= np.linalg.norm(handle_direction_vector)

    # Ensure surface_normal_vector points out of the surface
    if np.dot(handle_pointcloud_mean - surface_pointcloud_mean, surface_normal_vector) < 0:
        surface_normal_vector *= -1

    # Determine transforms between input and handle frames
    # Handle frame origin is closest point on the surface to handle_pointcloud_mean
    # Handle protrudes out of surface in +X direction
    # Length of handle runs along Z axis
    o_handle = handle_pointcloud_mean - vector_projection(handle_pointcloud_mean - surface_pointcloud_mean,surface_normal_vector)
    x_handle = surface_normal_vector.reshape((3, 1))
    z_handle = handle_direction_vector.reshape((3, 1))
    y_handle = np.cross(z_handle, x_handle, axisa=0, axisb=0, axisc=0)
    T_handle_to_input = np.array([[x_handle, y_handle, z_handle, o_handle],[0, 0, 0, 1]])
    return T_handle_to_input


def fit_line_to_pointcloud(pointcloud: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Fits a line to a pointcloud.
    :param pointcloud: np.ndarray of shape (n_points, 3)
    :return:
    np.ndarray of shape (3,) representing the line direction,
    and np.ndarray of shape (3,) representing the mean of the input pointcloud
    """
    pointcloud_mean = np.mean(pointcloud, axis=0, keepdims=True)
    pointcloud = pointcloud - pointcloud_mean
    pca = PCA(n_components=1)
    pca.fit(pointcloud)
    direction_vector = pca.components_[0]
    return direction_vector, pointcloud_mean.flatten()


def fit_plane_to_pointcloud(pointcloud: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    """
    Fits a plane to a pointcloud.
    :param pointcloud: np.ndarray of shape (n_points, 3)
    :return:
    np.ndarray of shape (3,) representing the normal vector to the plane,
    and np.ndarray of shape (3,) representing the mean of the input pointcloud
    """
    pointcloud_mean = np.mean(pointcloud, axis=0, keepdims=True)
    pointcloud = pointcloud - pointcloud_mean
    pca = PCA(n_components=3)
    pca.fit(pointcloud)
    normal_vector = pca.components_[2]
    return normal_vector, pointcloud_mean.flatten()


def vector_projection(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    """
    Calculates vector projection.
    :param a: np.ndarray of shape (3,)
    :param b: np.ndarray of shape (3,)
    :return: np.ndarray of shape (3,) representing a projected onto b
    """
    return (np.dot(a, b) / np.linalg.norm(b) ** 2) * b


if __name__ == "__main__":
    # TODO TESTS FOR EVERYTHING
    pass