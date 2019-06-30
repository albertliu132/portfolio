clear all;
clc;
tic;
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


% C = zeros(numFrames,3,'double');
% mov = zeros(numFrames,1,'uint8');

location = zeros(709,2);

% Create a MATLAB movie struct from the video frames.
for k = 1 : numFrames %numFrames
    
    disp(k);
    %      mov(k).cdata = im2uint8(imbinarize(vidFrames(:,:,:,k),0.06));
    mov(k).cdata = im2uint8(imcomplement(imbinarize(vidFrames(:,:,:,k),0.06)));
    
    this_frame = mov(k).cdata;
    [row,col] = find(this_frame(:,:,1));
    
    points = numel(row);
    new_row = zeros();
    new_col = zeros();
    
    
%--------------------------------------------------------------------------    
    %Older Code, tracks by finding average coordinate of white points
    % and it stops averaging points once it detects are large difference in
    % the next pixel coordinates, only works without noise
    
%     j = 1;
%     for i = 10 : points
%         if row(i)-row(i-1) < 50
%             new_row(j)= row(i);
%             new_col(j)= col(i);
%             j = j+1;
%         else
%             break;
%         end
%     end
%     
%     X = round(mean(new_row));
%     Y = round(mean(new_col));
%     object_center = [X Y];
%     location(k,:) = object_center;
    
%--------------------------------------------------------------------------

    %Newer Code, utilizes older code to determine first cetner point, then
    %it obly takes points into account which are within 20 pixels of its
    %center, works with noise
    
    if k == 1
        j = 1;
        for i = 2 : points
            if row(i)-row(i-1) < 50
                new_row(j)= row(i);
                new_col(j)= col(i);
                j = j+1;
            else
                break;
            end
        end
        
    
        X1 = round(mean(new_row));
        Y1 = round(mean(new_col));
        el = 1;
        el_row = zeros();
        el_col = zeros();
            
        for h = 1: points
            if abs(X1-row(h)) > 20 || abs(Y1 - col(h)) > 20
                mov(k).cdata(row(h), col(h),:) = [0 0 0];
                
                el_row(el) = row(h);
                el_col(el) = col(h);
                el = el + 1;
                
            end
        end
        
        
        this_frame = mov(k).cdata;
        [newer_row,newer_col] = find(this_frame(:,:,1));
        X2 = round(mean(newer_row));
        Y2 = round(mean(newer_col));
        object_center = [X2 Y2];
        location(k,:) = object_center;
        
    
    elseif k > 1
        j = 1;
        for i = 2: points
            if abs(row(i)-X2) <= 20 && abs(col(i)-Y2) <= 20
                new_row(j)= row(i);
                new_col(j)= col(i);
                j = j+1;  
            end  
        end
        
        X1 = round(mean(new_row));
        Y1 = round(mean(new_col));      
      
        for h = 1: points
            if abs(X1-row(h)) > 15 || abs(Y1 - col(h)) > 15
                mov(k).cdata(row(h), col(h),:) = [0 0 0]; 
            end
        end
        
        this_frame = mov(k).cdata;
        [newer_row,newer_col] = find(this_frame(:,:,1));
        X2 = round(mean(newer_row));
        Y2 = round(mean(newer_col));
        object_center = [X2 Y2];
        location(k,:) = object_center;
    end     

    %uncomment!!!
    for m = 1:7
        mov(k).cdata(object_center(1)-4+m, object_center(2),:) = [255 0 0];
    end
    
    for n = 1:7
        mov(k).cdata(object_center(1), object_center(2)-4+n,:) = [255 0 0];
    end
    
   
    mov(k).colormap = [];
    
end

% Plot of trajectory
 frame_num = numFrames; %709; numFrames;
 last_frame = im2uint8(imcomplement(imbinarize(vidFrames(:,:,:,frame_num),0.06)));
for n = 1:frame_num
    last_frame(location(n,1), location(n,2),:) = [255 0 0];
end

figure(1);
imshow(last_frame);

% figure(2);
% imshow(mov(2).cdata);

figure(2);
imshow(mov(1).cdata);

figure(3);
imshow(mov(372).cdata);

figure(4);
imshow(mov(743).cdata);

figure(5);
imshow(mov(1115).cdata);

figure(6);
imshow(mov(1486).cdata);

% Create a figure
hf = figure;

% Resize figure based on the video's width and height
set(hf, 'position', [150 150 readerobj.Width readerobj.Height])

toc;

% Playback movie once at the video's frame rate
movie(hf, mov, 1, readerobj.FrameRate);



% Old stuff not used
%
%        if k == 710
%             el = 1;
%             el_row = zeros();
%             el_col = zeros();
%             for h = 1: points
%                 if abs(X1-row(h)) || 20 && abs(Y1 - col(h)) > 20
%                     mov(k).cdata(row(h), col(h),:) = [0 0 0];
%                     el_row(el) = row(h);
%                     el_col(el) = col(h);
%                     el = el + 1;
%                     
%                 end
%             end
%         else
%             for p = 1: el-1
%                 mov(k).cdata(el_row(p), el_col(p),:) = [0 0 0];
%             end
%         end