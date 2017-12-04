function [ labels ] = getLabels( str )
%GETLABELS Summary of this function goes here
%   Detailed explanation goes here

commas = strfind(str,',');

N = length(commas) + 1;
labels = cell(N,1);

labels{1} = str(1:commas(1)-1);

for i=1:N-2
    if (str(commas(i)+1) == ' ')
        labels{i+1} = str(commas(i)+2:commas(i+1)-1);
    else
        labels{i+1} = str(commas(i)+1:commas(i+1)-1);
    end
    
end
if (str(commas(end)+1) == ' ')
    labels{end} = str(commas(end)+2:end-1);
else
    labels{end} = str(commas(end)+1:end-1);
end

end

