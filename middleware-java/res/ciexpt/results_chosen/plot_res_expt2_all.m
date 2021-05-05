function plot_res_expt2_all()
    plot_res_expt2_int_metric("ciexpt-casestudy2.res", 9, "casestudy2_completed_sweeps.pdf", "Number of completed sweeps", -0.5, 10);
    plot_res_expt2_hist(      "ciexpt-casestudy2.res", 7, "casestudy2_final_distance.pdf",   "Final distance from base at end", 0, 250);
    plot_res_expt2_hist(      "ciexpt-casestudy2.res", 5, "casestudy2_final_energy.pdf",   "Total energy on robots at end", 0, 2800);

%% This is for an optimal run
    plot_res_expt2_int_metric("ciexpt-casestudy2-optimal-repeated.res", 9, "casestudy2_optimal_repeated_completed_sweeps.pdf", "Number of completed sweeps", -0.5, 10);
    plot_res_expt2_hist(      "ciexpt-casestudy2-optimal-repeated.res", 8, "casestudy2_optimal_repeated_final_distance.pdf",   "Mean final distance from base at end", 0, 250);
    plot_res_expt2_hist(      "ciexpt-casestudy2-optimal-repeated.res", 5, "casestudy2_optimal_repeated_final_energy.pdf",   "Total energy on robots at end", 0, 2800);


%%  plot_res_expt2_int_metric("ciexpt-casestudy2-threshold500.res", 3, "casestudy2_completed_sweeps-500.pdf", "Number of completed sweeps", -0.5, 10);
%%  plot_res_expt2_hist(      "ciexpt-casestudy2-threshold500.res", 4, "casestudy2_final_distance-500.pdf",   "Final distance from base at end", 0, 250);
%%  plot_res_expt2_hist(      "ciexpt-casestudy2-threshold500.res", 6, "casestudy2_final_energy-500.pdf",   "Total energy on robots at end", 0, 2800);

%%  plot_res_expt2_int_metric("ciexpt-casestudy2-threshold250.res", 3, "casestudy2_completed_sweeps-250.pdf", "Number of completed sweeps", -0.5, 10);
%%  plot_res_expt2_hist(      "ciexpt-casestudy2-threshold250.res", 4, "casestudy2_final_distance-250.pdf",   "Final distance from base at end", 0, 250);
%%  plot_res_expt2_hist(      "ciexpt-casestudy2-threshold250.res", 6, "casestudy2_final_energy-250.pdf",   "Total energy on robots at end", 0, 2800);
endfunction
