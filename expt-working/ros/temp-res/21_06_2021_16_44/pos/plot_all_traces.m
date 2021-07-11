function plot_all_traces
  uav_1_files = glob("*_uav1");
  uav_2_files = glob("*_uav2");
  plot_traces_for_vehicle(uav_2_files, "UAV 2");
  plot_traces_for_vehicle(uav_1_files, "UAV 1");
end
