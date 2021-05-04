function plot_res_expt1
	missed_detections_col = 3;
	m = dlmread("ciexpt-casestudy1.res", ",");
	inds_even = 2*(1:length(m)/2)
	inds_odd = inds_even-1;
	even_rows = m(inds_even,:);
	odd_rows = m(inds_odd,:);
	advanced_missed = even_rows(:,missed_detections_col);
	standard_missed = odd_rows(:,missed_detections_col);

	clf();
        ui = unique(standard_missed);
        ys = histc(standard_missed,ui);
        bar(ui,ys);
        xlim([-0.5 7.5]);
        xlabel("Missed detection count");
        ylabel("Frequency");
        title("Missed detections for case study 1 under the standard CI");
	print("-dpdf", "casestudy1_missed_detections_standardonly.pdf");

	clf();
	subplot(2,1,1);

	ui = unique(standard_missed);
	ys = histc(standard_missed,ui);
	bar(ui,ys);
	xlim([-0.5 7.5]);
	xlabel("Missed detection count");
	ylabel("Frequency");
	title("Missed detections for case study 1 under the standard CI");

	subplot(2,1,2);
	ui = unique(advanced_missed);
	ya = hist(advanced_missed,ui);
	bar(ui,ya, 'FaceColor', 'green');
	xlim([-0.5 7.5]);
        xlabel("Missed detection count");
        ylabel("Frequency");
	title("Missed detections for case study 1 under the advanced CI");
	print("-dpdf", "casestudy1_missed_detections.pdf");
endfunction
