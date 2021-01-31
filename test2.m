clc
global i;
global k;
global T01;
global T02;
global T03;
global T1;
global area;

Imzero = zeros(420,560,3);      %计算图片背景
Im1 = rgb2gray(imread(['picture/','1.jpeg']));
%Im2 = rgb2gray(imread(['picture/','2.jpg']));
%Im3 = rgb2gray(imread(['picture/','3.jpg']));
%Im4 = rgb2gray(imread(['picture/','4.jpg']));
%Im5 = rgb2gray(imread(['picture/','5.jpg']));
%Imzero = (Im1)/5+(Im2)/5+(Im3)/5+(Im4)/5+(Im5)/5;
Imzero=Im1;
Imback = medfilt2(Imzero);
[MR,MC,Dim] = size(Imback);

%----卡尔曼滤波初始化----%
R  = [[0.2845,0.0045]',[0.0045,0.0455]'];        %观测噪声协方差矩阵
H  = [[1,0]',[0,1]',[0,0]',[0,0]'];              %观测转移矩阵
Q  = 0.01*eye(4);                                %状态噪声协方差矩阵（eye(4)指4*4单位阵）
P_init  = 100*eye(4);                                 %误差协方差
dt = 1;
A  = [[1,0,0,0]',[0,1,0,0]',[dt,0,1,0]',[0,dt,0,1]'];   %状态转移矩阵
g = 6;                      % pixels^2/time step
Bu = [0,0,0,g]';
kfinit=0;                   %初始指针
previous=0;
index=1;
%x=zeros(100,4);             %状态矩阵
x=zeros(400,4,100);
PK=zeros(400,16,100);

T1 = 50;
T2 = 0.7;
T3 = 0.7;

a1=1;
a2=222;
%起始默认坐标
X=0;Y=0;
for i = a1:3:a2
    %figure(1);
    subplot(121)
    if a1>205
        break
    end
    Im = (imread(['picture/',int2str(i), '.jpeg']));
    handles.savepic = Im;
    Im = rgb2gray(Im);
    imshow(Im);
    Imwork = Im;
    
    [stats,N,flag,foremm] = extract(Imwork,Imback,i);
    foremm = medfilt2(foremm);
    foremm=bwareaopen(foremm,100);%去除干扰小面积
    %imshow(foremm);
    se=strel('disk',25);       %生成圆形结构元素
    foremm=imdilate(foremm,se);        %用生成的结构元素对图像进行腐蚀
    subplot(122)
    imshow(foremm);
    strtitle = sprintf('当前第%d帧二值图',i);
    title(strtitle)
    
    if flag==0
        continue
    end
    j=1;
    prenum=1;
    [L,num1] = bwlabel(foremm,8);  %区域标记，
    %STATS = regionprops(L, 'all');
    MyRegionProps = regionprops(foremm,'Centroid','BoundingBox','Area');
    strtitle = sprintf('第%d帧',i);
    disp(strtitle)
    for k = 1:num1
        area(k)=MyRegionProps(k).Area;  %计算各区域的面积
        %subplot(121)
        %hold on;
        %plot(MyRegionProps(k).Centroid(1),MyRegionProps(k).Centroid(2),'r*');
        %%质心坐标
        %MyRegionProps(k).Centroid(1),
        %MyRegionProps(k).Centroid(2)
        
    end
    [S1,p]=max(area);
    %从stats中提取目标模板的参数
    centroid = stats(j).Centroid;
    radius = sqrt(stats(j).Area/pi);
    cc(i,j) = centroid(1);
    cr(i,j) = centroid(2);
    
    %将目标模板用绿色矩形标示出来
    subplot(121);
    imshow(Im);
    %strtitle = sprintf('当前第%d帧',i);
    %title(strtitle);
    rectangle('position',stats(j).BoundingBox,'edgecolor','g');
    
    
    
    %%%卡尔曼目标预测目标位置%%%%
    %利用特征进行匹配滤波器，从而进行卡尔曼预测
    if kfinit == 0
        xp = [MC/2,MR/2,0,0]';
        P = P_init;
    else
        %把当前帧的所有目标模板与前一帧的所有目标模板进行匹配
        for m = 1:previous
            distance = sqrt((cc(i,j)-cc(i-1,m))^2+(cr(i,j)-cr(i-1,m))^2);
            stats_pre(m).Area;
            stats_pre(m).Area;
            similarity = abs(stats(j).Area-stats_pre(m).Area)/stats_pre(m).Area;
            aa = stats(j).BoundingBox;
            width = aa(3);
            height = aa(4);
            ratio_j = width/height;
            aa = stats_pre(m).BoundingBox;
            width = aa(3);
            height = aa(4);
            ratio_m = width/height;
            wight_height_ratio = abs((ratio_j-ratio_m))/ratio_j;
            
            %匹配的依据是三个条件，如果同时符合三个条件，则继承相应的滤波器
            if(distance<T1&&similarity<T2&&wight_height_ratio<T3)
                xp=A*x(i-1,:,m)';
                P=reshape(PK(i-1,:,m),4,4);
                index=m;
                break;
            end
            if(m==previous)
                xp = [MC/2,MR/2,0,0]';
                P=P_init;
                index=previous+1;
            end
        end
    end
    %保存上一次数据，用于下次计算
    previous=prenum;
    stats_pre=stats;
    kfinit=1;
    PP = A*P*A' + Q;
    H1 = PP*H'*inv(H*PP*H'+R);
    x(i,:,index) = (xp + H1*([cc(i,j),cr(i,j)]' - H*xp))';
    P = (eye(4)-H1*H)*PP;
    PK(i,:,index)=reshape(P,1,16);
    hold on
    %将预测目标用红色矩形标示出来
    aa=stats(j).BoundingBox;
    width=aa(3);
    height=aa(4);
    %%rectangle('position',[x(i,1,index)-width/2,x(i,2,index)-height/2,width,height],'edgecolor','r');
    pause(0.3)
    strtitle = sprintf('第%d帧',i);
    %disp(strtitle)
    %disp('矩形宽为：')
    %width
    %disp('矩形高为：')
    %height
    %disp('矩形高为：')
    %%质心坐标
    disp('宽/高比例rate为：')
    rate=width/height
    %MyRegionProps(k).Centroid(1),MyRegionProps(k).Centroid(2)
    %%%判断姿态运动  
    disp('质心距离：')
    ddd=sqrt((MyRegionProps(k).Centroid(1)-X).^2+(MyRegionProps(k).Centroid(2)-Y).^2)
    if  ddd<3.5
        %disp('非行走')
        if  rate<0.7
            disp('站立，新陈代谢率：1.2')
            strtitle = sprintf('站立，新陈代谢率：1.2met');
            
        elseif  rate >0.7
            disp('伸展手臂，新陈代谢率：3.5')
            strtitle = sprintf('伸展手臂，新陈代谢率：3.5met');
        end
    elseif ddd>3.5
        disp('行走，新陈代谢率：2.0')
        strtitle = sprintf('行走，新陈代谢率：2.0met');
    end
    figure(1);
    imshow(Im);
    title(strtitle);
    rectangle('position',stats(j).BoundingBox,'edgecolor','g');
    
    %重新赋值新质心坐标给X Y变量
    X=MyRegionProps(k).Centroid(1);
    Y=MyRegionProps(k).Centroid(2);    
    
    
    if i>223
        break
    end
end
msgbox('处理完成！')
