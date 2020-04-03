function[]=Karin_2hrSeparate_EMZA(filename)

% Code for extracting the data from the ASCII files:
fileA = fopen(filename);
tempSampAtext = textscan(fileA, '%s',1); %#ok<NASGU>
tempSampAnum = textscan(fileA, '%f %f', 'delimiter', '"');
SampA = tempSampAnum{1,2}(:);
clear tempSampAtext tempSampAnum
fclose(fileA);
eelen=length(SampA);
clear SampA
sampRate = 200;
EPOCHSIZEOF2hrs = sampRate * 7200;

m = 1;
n = 1;
while m < eelen
    twoHrStartIndex(n) = m; %#ok<*AGROW>
    twoHrStart(n) = twoHrStartIndex(n)/200;
    if (twoHrStart(n) + 7200.001) > eelen/200
        twoHrEndIndex(n) = eelen;
        twoHrEnd(n) = eelen/200;
        m = eelen;
    else
        twoHrEndIndex(n) = twoHrStartIndex(n) + EPOCHSIZEOF2hrs - 1;
        twoHrEnd(n) = twoHrEndIndex(n)/200;
        m = 1 + twoHrEndIndex(n);
        n = n + 1;
    end
end
% Requests user input here for naming the timestamp file.
prompt={'Enter the filename you want to save it as: ( just the name )'};
def={'T_day1m'};
dlgTitle='Input for Timestamp utility';
lineNo=1;
answer=inputdlg(prompt,dlgTitle,lineNo,def);
filename=char(answer(1,:));
timestampfile=strcat(filename,'.xls');
fod=fopen(timestampfile,'w'); %Creates and opens user-named file
for i = 1:n
    interp2HrStartIndex(i) = 1;
    interp2HrEndIndex(i) = twoHrEndIndex(i) - twoHrStartIndex(i) + 1;
    if i==n
        ts_low(i) = twoHrStart(end);
        ts_high(i) = twoHrEnd(end);
        fprintf(fod,'%f\t %f\t %f\t %f\t %f\t %f\n', ts_low(i), ts_high(i), interp2HrStartIndex(i), interp2HrEndIndex(i), twoHrStart(i), twoHrEnd(i)); %writes the start and end times for the last epoch (can be <2hr)
    else
        ts_low(i) = twoHrStart(i);
        ts_high(i) = twoHrEnd(i);
        fprintf(fod,'%f\t %f\t %f\t %f\t %f\t %f\n', ts_low(i), ts_high(i), interp2HrStartIndex(i), interp2HrEndIndex(i), twoHrStart(i), twoHrEnd(i)); %Writes the value of the last index in the ts_low and ts_high vectors.
    end
end
fclose(fod);