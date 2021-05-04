function plot_res_expt1
	miss_col = 3;
	time_col = 4;
	markersize = 8;

	m = dlmread("ciexpt-casestudy1.res", ",");
	inds_even = 2*(1:length(m)/2)
	inds_odd = inds_even-1;
	even_rows = m(inds_even,:);
	odd_rows = m(inds_odd,:);
	advanced_missed = even_rows(:,miss_col);
	standard_missed = odd_rows(:,miss_col);
	advanced_timing = even_rows(:,time_col);
	standard_timing = odd_rows(:,time_col);

        clf();
        hold on;
        plot(standard_missed, standard_timing, "ro", "MarkerSize", markersize);
	%%        plot(advanced_missed, advanced_timing, "bx", "MarkerSize", markersize);
        %%        legend("Standard CI", "Advanced CI", 4);
        title("Missed detections versus sweep completion timing for case study 1 - standard CI only");
        print("-dpdf", "casestudy1_missed_vs_timing_standardonly.pdf");

	clf();
	hold on;
	plot(standard_missed, standard_timing, "ro", "MarkerSize", markersize);
	plot(advanced_missed, advanced_timing, "bx", "MarkerSize", markersize);
	legend("Standard CI", "Advanced CI", 4);
	title("Missed detections versus sweep completion timing for case study 1");
	print("-dpdf", "casestudy1_missed_vs_timing.pdf");
endfunction
