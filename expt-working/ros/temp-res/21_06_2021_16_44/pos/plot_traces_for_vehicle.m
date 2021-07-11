function plot_traces_for_vehicle(vehicle_files, vehicle_name)
  cols = {"-bx", "-rx", "-gx", "-bx.", ".k"}
  hold on;
  xcol = 3;
  ycol = 4;
  zcol = 5;

  clf;
  for i = 1:length(vehicle_files)
    hold on;
    filename = vehicle_files{i}
    colnum = mod(i, length(cols))+1;
    m = dlmread(filename,",");
    if (length(m) > 0)
      plot3(m(:,xcol), m(:,ycol), m(:,zcol), cols{colnum});
#     plot(m(:,2), m(:,xcol))
    end
    xlabel("X");
    ylabel("Y");
    zlabel("Z");
    title(["Vehicle name - ", vehicle_name])
  end
endfunction
