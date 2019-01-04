function bodyinfo = read_skeleton_file1()%filename, id)
% Reads an .skeleton file from "NTU RGB+D 3D Action Recognition Dataset".
% 
% Argrument:
%   filename: full adress and filename of the .skeleton file.
%
% For further information please refer to:
%   NTU RGB+D dataset's webpage: 
%       http://rose1.ntu.edu.sg/Datasets/actionRecognition.asp
%   NTU RGB+D dataset's github page: 
%        https://github.com/shahroudy/NTURGB-D
%   CVPR 2016 paper: 
%       Amir Shahroudy, Jun Liu, Tian-Tsong Ng, and Gang Wang, 
%       "NTU RGB+D: A Large Scale Dataset for 3D Human Activity Analysis", 
%       in IEEE Conference on Computer Vision and Pattern Recognition (CVPR), 2016
%
% For more details about the provided data, please refer to:
%   https://msdn.microsoft.com/en-us/library/dn799271.aspx
%   https://msdn.microsoft.com/en-us/library/dn782037.aspx

% fileid = fopen('/Users/Emily/Downloads/nturgb+d_skeletons_clapping);
Files=dir('./nturgb+d_skeletons_clapping/*.skeleton');
mat1=[];
mat2=[];
selectedjoint=[4,5,6,7,8,9,10,11,12,22,23,24,25];
gamma = 0.95;
output=[];
for k=1:900
    mat1_each=[];
    mat2_each=[];
    file = strcat(Files(k).folder,'/',Files(k).name);
    fileid = fopen(file);
    %     disp(file);
    framecount = fscanf(fileid,'%d',1); % no of the recorded frames
    %     disp(framecount);
    bodyinfo=[]; % to store multiple skeletons per frame
    prev=[];
    for f=1:framecount
        bodycount = fscanf(fileid,'%d',1); % no of observerd skeletons in current frame
        for b=1:bodycount % only 1 in our example
            clear body;
            body.bodyID = fscanf(fileid,'%ld',1); % tracking id of the skeleton
            arrayint = fscanf(fileid,'%d',6); % read 6 integers
            body.clipedEdges = arrayint(1);
            body.handLeftConfidence = arrayint(2);
            body.handLeftState = arrayint(3);
            body.handRightConfidence = arrayint(4);
            body.handRightState = arrayint(5);
            body.isResticted = arrayint(6);
            lean = fscanf(fileid,'%f',2);
            body.leanX = lean(1);
            body.leanY = lean(2);
            body.trackingState = fscanf(fileid,'%d',1);

            body.jointCount = fscanf(fileid,'%d',1); % no of joints (25)
            joints=[];
            for j=1:body.jointCount
                jointinfo = fscanf(fileid,'%f',11);
                joint=[];

                % 3D location of the joint j
                joint.x = jointinfo(1);
                joint.y = jointinfo(2);
                joint.z = jointinfo(3);

                % 2D location of the joint j in corresponding depth/IR frame
                joint.depthX = jointinfo(4);
                joint.depthY = jointinfo(5);

                % 2D location of the joint j in corresponding RGB frame
                joint.colorX = jointinfo(6);
                joint.colorY = jointinfo(7);

                % The orientation of the joint j
                joint.orientationW = jointinfo(8);
                joint.orientationX = jointinfo(9);
                joint.orientationY = jointinfo(10);
                joint.orientationZ = jointinfo(11);

                % The tracking state of the joint j
                joint.trackingState = fscanf(fileid,'%d',1);

                body.joints(j)=joint;
            end
            %disp(body.joints(3).x) %neck
            %disp(body.joints(13)) %left hip
            %disp(body.joints(17)) %right hip
            bodyinfo(f).bodies(b)=body;
        end
        % coordinate transformation
        u1_temp = [body.joints(13).x-body.joints(3).x body.joints(13).y-body.joints(3).y body.joints(13).z-body.joints(3).z];
        u1 = u1_temp/norm(u1_temp);
        u2_temp = [body.joints(17).x-body.joints(3).x body.joints(17).y-body.joints(3).y body.joints(17).z-body.joints(3).z];
        u3 = cross(u1,u2_temp)/norm(cross(u1,u2_temp));
        u2 = cross(u1,u3);
        s = norm(u1_temp);
        for j=1:body.jointCount
            new_joint = [];
            new_v = [body.joints(j).x-body.joints(3).x body.joints(j).y-body.joints(3).y body.joints(j).z-body.joints(3).z];
            new_joint.x = (1/s)*dot(new_v,u1);
            new_joint.y = (1/s)*dot(new_v,u2);
            new_joint.z = (1/s)*dot(new_v,u3);
            body.new_joints(j)=new_joint;
        end
        if f == 1
            prev=body.new_joints;
        end
        % calculate difference between t and t-1
        mat1_row = [];
        for j=1:body.jointCount
            diff = [];
            diff.x = body.new_joints(j).x;
            diff.y = body.new_joints(j).y;
            diff.z = body.new_joints(j).z;
            diff.dx = body.new_joints(j).x-prev(j).x;
            diff.dy = body.new_joints(j).y-prev(j).y;
            diff.dz = body.new_joints(j).z-prev(j).z;
            body.state(j) = diff;
            S(f,j) = diff;
            if ismember(j,selectedjoint)
                if f>=3
                    temp_mat1=[diff.x,diff.y,diff.z,diff.dx,diff.dy,diff.dz];
                    mat1_row = [mat1_row,temp_mat1];
                end
            end
        end
        if f>=3
            mat1_each=[mat1_each;mat1_row];
        end
        %disp(class(body.state));
        %disp(body.diffs);
        %body.state(f) = body.diffs;
        %disp(body.new_joints(13))
        prev=body.new_joints;
    end
    % for f=1:framecount
    %     for j=1:25
    %         disp(S(f,j));
    %     end
    % % end

    for f=3:framecount-1
        mat2_row=[];
        for j1=1:13
            j = selectedjoint(j1);
            a=[];
            a.ax=S(f+1,j).dx-S(f,j).dx;
            a.ay=S(f+1,j).dy-S(f,j).dy; 
            a.az=S(f+1,j).dz-S(f,j).dz;
            temp_mat2 = [a.ax,a.ay,a.az];
            mat2_row = [mat2_row,temp_mat2];
    %         A(f,j)=a;
        end
        mat2_each=[mat2_each;mat2_row];
    end
    output_each=[];
    mat1_each(end,:) = [];

    for f=1:size(mat1_each,1)
        output_each(f)=power(gamma,size(mat1_each,1)-f);
    end


    fclose(fileid);
    %mat1=[mat1;mat1_each];
    %mat2=[mat2;mat2_each];
    %output=[output,output_each];

    %[m,n] = size(mat1_each);
    %disp(m)
    %disp(n)
    % disp(mat1);

    %[m,n] = size(mat2_each);
    %disp(m);
    %disp(n);
    % disp(mat2);

    %[m,n] = size(output_each);
    %disp(m);
    %disp(n);
    % disp(output_each)

    save(strcat('matrics',int2str(k),'.mat'),'mat1_each','mat2_each','output_each');
end
end