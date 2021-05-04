function plot_res_expt2_int_metric(resfile, metric_col, filename, metric_name)
	m = dlmread(resfile, ",");
        inds_even = 2*(1:size(m)/2)
	inds_odd = inds_even-1;
	even_rows = m(inds_even,:)
	odd_rows = m(inds_odd,:)
	advanced_missed = even_rows(:,metric_col);
	standard_missed = odd_rows(:,metric_col);
	clf();
	subplot(2,1,1);

	ui = unique(standard_missed);
	ys = histc(standard_missed,ui);
	bar(ui,ys);
	xlim([-0.5 5.5]);
	xlabel("Missed detection count");
	ylabel("Frequency");
        title({metric_name "Missed detections for case study 1 under the standard CI"});

	subplot(2,1,2);
	ui = unique(advanced_missed);
	ya = hist(advanced_missed,ui);
	bar(ui,ya, 'FaceColor', 'green');
	xlim([-0.5 5.5]);
        xlabel("Missed detection count");
        ylabel("Frequency");
	title({metric_name "Missed detections for case study 1 under the energy tracking CI"});
	print("-dpdf", filename);
endfunction
