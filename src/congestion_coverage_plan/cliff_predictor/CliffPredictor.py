import matplotlib.pyplot as plt
from tqdm import *

from congestion_coverage_plan.cliff_predictor.TrajectoryPredictor import TrajectoryPredictor
import congestion_coverage_plan.utils.Plotter as Plotter
import congestion_coverage_plan.utils.dataset_utils as dataset_utils
from datetime import datetime
import numpy as np

class CliffPredictor:

    def __init__(self, dataset, map_file, mod_file, observed_tracklet_length, start_length, planning_horizon, beta, sample_radius, delta_t, method, fig_size, ground_truth_data_file):
        self.map_file = map_file
        self.mod_file = mod_file
        self.ground_truth_data_file = ground_truth_data_file
        # self.result_file = result_file
        self.observed_tracklet_length = observed_tracklet_length
        self.start_length = start_length
        self.planning_horizon = planning_horizon
        self.beta = beta
        self.sample_radius = sample_radius
        self.delta_t = delta_t
        self.method = method
        self.dataset = dataset
        self.fig_size = fig_size # [xmin, xmax, ymin, ymax]
        self.cliff_map_data = dataset_utils.read_cliff_map_data(mod_file)
        # print(fig_size)
        # self.human_traj_data = utils.read_iit_human_traj_data(self.ground_truth_data_file)
    def get_all_person_id(self, data):
        person_id_list = list(data.person_id.unique())
        return person_id_list




    def get_human_traj_data_by_person_id(self, human_traj_origin_data, person_id):
        human_traj_data_by_person_id = human_traj_origin_data.loc[human_traj_origin_data['person_id'] == person_id]
        human_traj_array = human_traj_data_by_person_id[["time", "x", "y", "velocity", "motion_angle"]].to_numpy()
        return human_traj_array

    def display_cliff_map_and_save(self):
        # fig, ax = plt.subplot(111, facecolor='grey')
        img = plt.imread(self.map_file)
        plt.imshow(img, cmap='gray', vmin=0, vmax=255, extent=self.fig_size)
        Plotter.plot_cliff_map(self.cliff_map_data)
        name = self.mod_file.split("/")[-1].split(".")[0]
        plt.savefig(f"{name}_cliff_map.png", dpi=400)

        plt.show()

    def display_cliff_map_with_prediction(self, all_predicted_trajectory_list, planning_horizon = 50):
        fig, ax = plt.subplot(111, facecolor='grey')
        img = plt.imread(self.map_file)
        plt.imshow(img, cmap='gray', vmin=0, vmax=255, extent=self.fig_size)
        Plotter.plot_cliff_map(self.cliff_map_data)
        Plotter.plot_all_predicted_trajs(all_predicted_trajectory_list, self.observed_tracklet_length)
        for predicted_people in all_predicted_trajectory_list:

            # plot_figures.plot_observed_tracklet(predicted_people, self.observed_tracklet_length)
            # print("all_predicted_trajectory_list", predicted_people)
            # print(len(predicted_people))
            for predicted_traj in predicted_people:
                # print("predicted_traj", len(predicted_traj))
                if len(predicted_traj) < planning_horizon:
                    continue
                final_pose = predicted_traj[planning_horizon - 1]
                # initial_pose = predicted_traj[0]
                # print("initial_pose", initial_pose)
                # plt.scatter(initial_pose[1], initial_pose[2], marker='D', alpha=1, color="r", s=100, label="Initial position")
                plt.scatter(final_pose[1], final_pose[2], marker='D', alpha=1, color="b", s=100, label="Predicted position")

        plt.show()

    # here person_positions is a list of person positions
    # it is a dictionary composed of: {person_id: [{time: t, x: x, y: y, velocity: v, motion_angle: a}, ...]}
    def predict_positions(self, person_positions, planning_horizon = 50):
        # print("person_positions", person_positions)
        if planning_horizon:
            self.planning_horizon = planning_horizon
        all_predictions = []
        # human_traj_data = utils.read_iit_human_traj_data("dataset/iit/iit.csv)
        for person_id in person_positions.keys():
            time_for_one_prediction = datetime.now()
            traj = person_positions[person_id]
            # print("person_positions", person_positions)
            # sort the trajectory by time
            # print("traj", traj)
            traj = sorted(traj, key=lambda x: float(x[0]))
            human_traj_data_by_person_id = np.array([[pose[0], pose[1], pose[2], pose[3], pose[4]] for pose in traj])

            # human_traj_data_by_person_id = self.get_human_traj_data_by_person_id(human_traj_data, person_id)

            
            # print("human_traj_data_by_person_id", human_traj_data_by_person_id)
            trajectory_predictor = TrajectoryPredictor(
                cliff_map_origin_data=self.cliff_map_data,
                human_traj_origin_data=human_traj_data_by_person_id,
                person_id=person_id,
                start_length=self.start_length,
                observed_tracklet_length=self.observed_tracklet_length,
                max_planning_horizon=self.planning_horizon,
                beta=self.beta,
                sample_radius=self.sample_radius,
                delta_t=self.delta_t,
                result_file="out.txt"
            )
            # print("trajectory_predictor.check_human_traj_data()", trajectory_predictor.check_human_traj_data())
            if not trajectory_predictor.check_human_traj_data():
                # print("trajectory_predictor.check_human_traj_data() failed")
                continue

            if self.method == dataset_utils.Method.MoD:
                all_predicted_trajectory_list = trajectory_predictor.predict_one_human_traj_mod()
            elif self.method == dataset_utils.Method.CVM:
                try:
                    all_predicted_trajectory_list = trajectory_predictor.predict_one_human_traj_cvm()
                except Exception as e:
                    print("Error occurred while predicting trajectory:", e)
            all_predictions.append(all_predicted_trajectory_list)
            # print("time_for_one_prediction", datetime.now() - time_for_one_prediction)
            # self.display_cliff_map(all_predicted_trajectory_list)
        # print("Ended prediction")
        return all_predictions

    def evaluate(self, human_traj_data, predicted_traj):
        for track in predicted_traj:
            for index, pose in enumerate(track):
                pass
