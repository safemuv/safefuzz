function plot_res_expt2_int_metric(metric_col, filename, metric_name, xmin, xmax)
	m = dlmread("ciexpt-casestudy2.res", ",");
inds_even = 2*(1:size(m)/2);
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
        xlim([xmin xmax]);
	xlabel(metric_name);
	ylabel("Frequency");
        title({metric_name " for case study 2 under the standard CI"});

	subplot(2,1,2);
	ui = unique(advanced_missed);
	ya = hist(advanced_missed,ui);
	bar(ui,ya, 'FaceColor', 'green');
        xlim([xmin xmax]);
        xlabel(metric_name);
        ylabel("Frequency");
	title({metric_name " for case study 2 under the energy tracking CI"});
	print("-dpdf", filename);
endfunction
