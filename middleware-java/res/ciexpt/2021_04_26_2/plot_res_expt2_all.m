function plot_res_expt2_all()
  plot_res_expt2_int_metric("ciexpt-casestudy2-threshold750.res", 4, "casestudy2_completed_sweeps-750.pdf", "Number of completed sweeps", -0.5, 10);
  plot_res_expt2_hist(      "ciexpt-casestudy2-threshold750.res", 9, "casestudy2_final_distance-750.pdf",   "Final distance from base at end", 0, 250);
  plot_res_expt2_hist(      "ciexpt-casestudy2-threshold750.res", 3, "casestudy2_final_energy-750.pdf",   "Total energy on robots at end", 0, 2800);

  plot_res_expt2_int_metric("ciexpt-casestudy2-threshold500.res", 4, "casestudy2_completed_sweeps-500.pdf", "Number of completed sweeps", -0.5, 10);
  plot_res_expt2_hist(      "ciexpt-casestudy2-threshold500.res", 9, "casestudy2_final_distance-500.pdf",   "Final distance from base at end", 0, 250);
  plot_res_expt2_hist(      "ciexpt-casestudy2-threshold500.res", 3, "casestudy2_final_energy-500.pdf",   "Total energy on robots at end", 0, 2800);

  plot_res_expt2_int_metric("ciexpt-casestudy2-threshold250.res", 4, "casestudy2_completed_sweeps-250.pdf", "Number of completed sweeps", -0.5, 10);
  plot_res_expt2_hist(      "ciexpt-casestudy2-threshold250.res", 9, "casestudy2_final_distance-250.pdf",   "Final distance from base at end", 0, 250);
  plot_res_expt2_hist(      "ciexpt-casestudy2-threshold250.res", 3, "casestudy2_final_energy-250.pdf",   "Total energy on robots at end", 0, 2800);
endfunction
