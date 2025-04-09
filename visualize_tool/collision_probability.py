
import numpy as np
from scipy.stats import multivariate_normal, mvn
import commonroad_dc.pycrcc as pycrcc

def get_collision_probability_fast(traj, predictions: dict, vehicle_params):

    obstacle_ids = list(predictions.keys())
    collision_prob_dict = {}

    # get the current positions array of the ego vehicles
    ego_pos = np.stack((traj.cartesian.x, traj.cartesian.y), axis=-1)

    # offset between vehicle center point and corner point
    offset = np.array([vehicle_params.length / 6, vehicle_params.width / 2])

    for obstacle_id in obstacle_ids:
        mean_list = predictions[obstacle_id]['pos_list']
        cov_list = predictions[obstacle_id]['cov_list']
        yaw_list = predictions[obstacle_id]['orientation_list']
        length = predictions[obstacle_id]['shape']['length']
        probs = []

        # mean distance calculation
        # determine the length of arrays
        min_len = min(len(traj.cartesian.x), len(mean_list))

        # adjust array of the ego vehicles
        # ego_pos_array = np.stack((traj.x[1:min_len], traj.y[1:min_len]), axis=-1)
        ego_pos_array = ego_pos[1:min_len]

        # get the positions array of the front and the back of the obsatcle vehicle
        mean_deviation_array = np.stack((np.cos(yaw_list[1:min_len]), np.sin(yaw_list[1:min_len])), axis=-1) * length / 2
        mean_array = np.array(mean_list[:min_len - 1])
        mean_front_array = mean_array + mean_deviation_array
        mean_back_array = mean_array - mean_deviation_array

        # total_mean_array = np.stack((mean_array, mean_front_array, mean_back_array))
        total_mean_array = np.array([mean_array, mean_front_array, mean_back_array])

        for i in range(1, len(traj.cartesian.x)):
            # only calculate probability as the predicted obstacle is visible
            if i < len(mean_list):
                # directly use previous bool result for the if statements

                cov = cov_list[i - 1]
                # if the covariance is a zero matrix, the prediction is
                # derived from the ground truth
                # a zero matrix is not singular and therefore no valid
                # covariance matrix
                allcovs = [cov[0][0], cov[0][1], cov[1][0], cov[1][1]]
                if all(covi == 0 for covi in allcovs):
                    cov = [[0.1, 0.0], [0.0, 0.1]]

                prob = 0.0

                # the occupancy of the ego vehicle is approximated by three
                # axis aligned rectangles
                # get the center points of these three rectangles
                center_points = get_center_points_for_shape_estimation(
                    length=vehicle_params.length,
                    width=vehicle_params.width,
                    orientation=traj.cartesian.theta[i],
                    pos=ego_pos_array[i - 1],
                )

                # upper_right and lower_left points
                center_points = np.array(center_points)

                upper_right = center_points + offset
                lower_left = center_points - offset

                # use mvn.mvnun to calculate multivariant cdf
                # the probability distribution consists of the partial
                # multivariate normal distributions
                # this allows to consider the length of the predicted
                # obstacle
                # consider every distribution

                for mu in total_mean_array[:, i - 1]:
                    for center_point_index in range(len(center_points)):
                        prob += mvn.mvnun(lower_left[center_point_index], upper_right[center_point_index], mu, cov)[0]



                # 使用 PDF 方法计算碰撞概率
                # for mu in total_mean_array[:, i - 1]:
                #     rv = multivariate_normal(mean=mu, cov=cov)
                #     for center_point_index in range(len(center_points)):
                #         # 计算矩形区域的面积 (近似)
                #         rect_area = (upper_right[center_point_index] - lower_left[center_point_index]).prod()
                #
                #         # 计算中心点的 PDF 值并乘以面积近似概率
                #         prob += rv.pdf(center_points[center_point_index]) * rect_area

                # for mu in total_mean_array[:, i - 1]:
                #     for center_point_index in range(len(center_points)):
                #         cdf_value = multivariate_normal.cdf(upper_right[center_point_index], mean=mu, cov=cov) - \
                #                     multivariate_normal.cdf(lower_left[center_point_index], mean=mu, cov=cov)
                #         prob += cdf_value

            else:
                prob = 0.0
            # divide by 3 since 3 probability distributions are added up and
            # normalize the probability
            probs.append(prob / 3)

        collision_prob_dict[obstacle_id] = np.array(probs)

    return collision_prob_dict


def get_center_points_for_shape_estimation(
    length: float, width: float, orientation: float, pos: np.array
):
    """
    Get the 3 center points for axis aligned rectangles.

    Get the 3 center points for axis aligned rectangles that approximate an
    orientated rectangle.

    Args:
        length (float): Length of the oriented rectangle.
        width (float): Width of the oriented rectangle.
        orientation (float): Orientation of the oriented rectangle.
        pos (np.array): Center of the oriented rectangle.

    Returns:
        [np.array]: Array with 3 entries, every entry holds the center of one
            axis aligned rectangle.
    """
    # create the oriented rectangle
    obj = pycrcc.RectOBB(length / 2, width / 2, orientation, pos[0], pos[1])

    center_points = []
    obj_center = obj.center()
    # get the directional vector
    r_x = obj.r_x()
    # get the length
    a_x = obj.local_x_axis()
    # append three points (center point of the rectangle, center point of the
    # front third of the rectangle and center point of the back third of the
    # rectangle)
    center_points.append(obj_center)
    center_points.append(obj_center + r_x * (2 / 3) * a_x)
    center_points.append(obj_center - r_x * (2 / 3) * a_x)

    return center_points




