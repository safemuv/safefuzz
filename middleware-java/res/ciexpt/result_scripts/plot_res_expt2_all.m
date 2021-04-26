function plot_res_expt2_all()
  plot_res_expt2_int_metric(3, "casestudy2_completed_sweeps.pdf", "Number of completed sweeps", -0.5, 6);
plot_res_expt2_hist(4, "casestudy2_final_distance.pdf",   "Final distance from base at end", 0, 250);
plot_res_expt2_hist(6, "casestudy2_final_energy.pdf",   "Total energy on robots at end", 0, 1000);
endfunction
