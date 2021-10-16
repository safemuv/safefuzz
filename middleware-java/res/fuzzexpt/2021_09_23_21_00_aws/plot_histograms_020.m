function plot_histograms
  m = dlmread("fuzzexpt-set-of-solutions-020.res")
  m = m(2:length(m),:)
  hist(m(:,3),24);
  title("Histogram of Inner region violations for repeats of one solution");
  pause();
  hist(m(:,4),24);
  title("Histogram of Speed violations for repeats of one solution");
  pause();
  hist(m(:,5),24);
  title("Histogram of Outer region violations for repeats of one solution");
endfunction
