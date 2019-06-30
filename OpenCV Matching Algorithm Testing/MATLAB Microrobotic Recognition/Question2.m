clear all;
clc;
% Construct a multimedia reader object associated with file - change the
% filename to the video you have been assigned
readerobj = VideoReader('MIE505Video4.mp4');

%load in goalpoints text file - change the name of the file to the one you
%have been assigned
fileID = fopen('MIE505_Goalpoints_Video4.txt');
waypoints = cell2mat(textscan(fileID,'%f %f'));
fclose(fileID);

% Read in all video frames.
vidFrames = read(readerobj);

% Get the number of frames.
numFrames = get(readerobj, 'NumberOfFrames');

% Create a MATLAB movie struct from the video frames.
for k = 1 : numFrames
     mov(k).cdata = vidFrames(:,:,:,k);
     mov(k).bmdata = im2uint8(imcomplement(imbinarize(vidFrames(:,:,:,k),0.04)));
     mov(k).colormap = [];
     currentBMFrame = mov(k).bmdata;
     
     [row,col] = find(currentBMFrame(:,:,1));
     numWhite = numel(row);
     filteredRow = zeros();
     filteredCol = zeros();
     
     if k == 1
         j = 1;
         for i = 2:numWhite
            if ((row(i) - row(i-1)) < 30)||(col(i) - col(i-1) < 30)
                filteredRow(j) = row(i);
                filteredCol(j) = col(i);
                j = j+1;
            else
                break;
            end
         end
     X = round(mean(filteredRow));
     Y = round(mean(filteredCol));
     objectCenter = [X Y];
     location(k,:) = objectCenter;
     for i = 1:numel(filteredCol)
        mov(k).cdata(filteredRow(i), filteredCol(i), :) = [0 0 255];
     end    
     for m = 1:7
        mov(k).cdata(objectCenter(1)-4+m, objectCenter(2),:) = [255 0 0];
        mov(k).bmdata(objectCenter(1)-4+m, objectCenter(2),:) = [255 0 0];
     end
     for m = 1:7
        mov(k).cdata(objectCenter(1), objectCenter(2)-4+m,:) = [255 0 0];
        mov(k).bmdata(objectCenter(1), objectCenter(2)-4+m,:) = [255 0 0];
     end
     else
        j = 1;
        for i = 2:numWhite
            if abs(row(i)-X) <= 20 && abs(col(i)-Y) <= 20
                filteredRow(j)= row(i);
                filteredCol(j)= col(i);
                j = j+1;
            else
                mov(k).bmdata(row(i), col(i), :) = 0;
            end
        end
        X = round(mean(filteredRow));
        Y = round(mean(filteredCol));
        objectCenter = [X Y];
        location(k,:) = objectCenter;
     end
     for m = 1:7
        mov(k).cdata(objectCenter(1)-4+m, objectCenter(2),:) = [255 0 0];
        mov(k).bmdata(objectCenter(1)-4+m, objectCenter(2),:) = [255 0 0];
     end
     for m = 1:7
        mov(k).cdata(objectCenter(1), objectCenter(2)-4+m,:) = [255 0 0];
        mov(k).bmdata(objectCenter(1), objectCenter(2)-4+m,:) = [255 0 0];
     end
end

% Create a figure
hf = figure; 

% Resize figure based on the video's width and height
set(hf, 'position', [150 150 readerobj.Width readerobj.Height])

% Playback movie once at the video's frame rate
movie(hf, mov, 1, readerobj.FrameRate);

