%extra(7)
%a = extra(7,3)
  
% Initialize handles for subplots
plotHandles = struct();
plotHandles.h1 = struct('lines', {cell(1, 5)}, 'closest_point', []);
plotHandles.h2 = struct('lines', {cell(1, 5)}, 'closest_point', []);
plotHandles.h3 = struct('lines', {cell(1, 5)}, 'closest_point', []);

% Update data in a loop
subplots = {'h1', 'h2', 'h3'};

for i=1:length(subplots)
    subplot_handle = subplots{i};
    color_set = {[0 0.4470 0.7410], [0.6350 0.0780 0.1840], [0.4940 0.1840 0.5560], [0.4660 0.6740 0.1880], [0.8500 0.3250 0.0980]};

    for j=1:5
        plotHandles.(subplot_handle).lines{j} = plot3(NaN, NaN, NaN, '-', 'LineWidth', 8, 'Color', color_set{j});
    end
end

disp(plotHandles.h1)
