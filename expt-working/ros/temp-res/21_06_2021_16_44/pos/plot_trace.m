function plot_trace(filename, vehicle_name, graphic)
  xcol = 3;
  ycol = 4;
  zcol = 5;
  m = dlmread(filename,",");
  if (length(m) > 0)
    hold on;
    plot3(m(:,xcol), m(:,ycol), m(:,zcol), graphic);
  end
endfunction
