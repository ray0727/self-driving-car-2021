from argoverse.evaluation.competition_util import generate_tracking_zip

output_dir = 'results/'
data_dir = "/home/ray/self-driving-car-2021/catkin_ws/src/final/results/results_tracking_test_cbgs"
generate_tracking_zip(data_dir,output_dir)