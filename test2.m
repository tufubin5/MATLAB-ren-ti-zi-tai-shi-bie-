clc
global i;
global k;
global T01;
global T02;
global T03;
global T1;
global area;

Imzero = zeros(420,560,3);      %����ͼƬ����
Im1 = rgb2gray(imread(['picture/','1.jpeg']));
%Im2 = rgb2gray(imread(['picture/','2.jpg']));
%Im3 = rgb2gray(imread(['picture/','3.jpg']));
%Im4 = rgb2gray(imread(['picture/','4.jpg']));
%Im5 = rgb2gray(imread(['picture/','5.jpg']));
%Imzero = (Im1)/5+(Im2)/5+(Im3)/5+(Im4)/5+(Im5)/5;
Imzero=Im1;
Imback = medfilt2(Imzero);
[MR,MC,Dim] = size(Imback);

%----�������˲���ʼ��----%
R  = [[0.2845,0.0045]',[0.0045,0.0455]'];        %�۲�����Э�������
H  = [[1,0]',[0,1]',[0,0]',[0,0]'];              %�۲�ת�ƾ���
Q  = 0.01*eye(4);                                %״̬����Э�������eye(4)ָ4*4��λ��
P_init  = 100*eye(4);                                 %���Э����
dt = 1;
A  = [[1,0,0,0]',[0,1,0,0]',[dt,0,1,0]',[0,dt,0,1]'];   %״̬ת�ƾ���
g = 6;                      % pixels^2/time step
Bu = [0,0,0,g]';
kfinit=0;                   %��ʼָ��
previous=0;
index=1;
%x=zeros(100,4);             %״̬����
x=zeros(400,4,100);
PK=zeros(400,16,100);

T1 = 50;
T2 = 0.7;
T3 = 0.7;

a1=1;
a2=222;
%��ʼĬ������
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
    foremm=bwareaopen(foremm,100);%ȥ������С���
    %imshow(foremm);
    se=strel('disk',25);       %����Բ�νṹԪ��
    foremm=imdilate(foremm,se);        %�����ɵĽṹԪ�ض�ͼ����и�ʴ
    subplot(122)
    imshow(foremm);
    strtitle = sprintf('��ǰ��%d֡��ֵͼ',i);
    title(strtitle)
    
    if flag==0
        continue
    end
    j=1;
    prenum=1;
    [L,num1] = bwlabel(foremm,8);  %�����ǣ�
    %STATS = regionprops(L, 'all');
    MyRegionProps = regionprops(foremm,'Centroid','BoundingBox','Area');
    strtitle = sprintf('��%d֡',i);
    disp(strtitle)
    for k = 1:num1
        area(k)=MyRegionProps(k).Area;  %�������������
        %subplot(121)
        %hold on;
        %plot(MyRegionProps(k).Centroid(1),MyRegionProps(k).Centroid(2),'r*');
        %%��������
        %MyRegionProps(k).Centroid(1),
        %MyRegionProps(k).Centroid(2)
        
    end
    [S1,p]=max(area);
    %��stats����ȡĿ��ģ��Ĳ���
    centroid = stats(j).Centroid;
    radius = sqrt(stats(j).Area/pi);
    cc(i,j) = centroid(1);
    cr(i,j) = centroid(2);
    
    %��Ŀ��ģ������ɫ���α�ʾ����
    subplot(121);
    imshow(Im);
    %strtitle = sprintf('��ǰ��%d֡',i);
    %title(strtitle);
    rectangle('position',stats(j).BoundingBox,'edgecolor','g');
    
    
    
    %%%������Ŀ��Ԥ��Ŀ��λ��%%%%
    %������������ƥ���˲������Ӷ����п�����Ԥ��
    if kfinit == 0
        xp = [MC/2,MR/2,0,0]';
        P = P_init;
    else
        %�ѵ�ǰ֡������Ŀ��ģ����ǰһ֡������Ŀ��ģ�����ƥ��
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
            
            %ƥ����������������������ͬʱ����������������̳���Ӧ���˲���
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
    %������һ�����ݣ������´μ���
    previous=prenum;
    stats_pre=stats;
    kfinit=1;
    PP = A*P*A' + Q;
    H1 = PP*H'*inv(H*PP*H'+R);
    x(i,:,index) = (xp + H1*([cc(i,j),cr(i,j)]' - H*xp))';
    P = (eye(4)-H1*H)*PP;
    PK(i,:,index)=reshape(P,1,16);
    hold on
    %��Ԥ��Ŀ���ú�ɫ���α�ʾ����
    aa=stats(j).BoundingBox;
    width=aa(3);
    height=aa(4);
    %%rectangle('position',[x(i,1,index)-width/2,x(i,2,index)-height/2,width,height],'edgecolor','r');
    pause(0.3)
    strtitle = sprintf('��%d֡',i);
    %disp(strtitle)
    %disp('���ο�Ϊ��')
    %width
    %disp('���θ�Ϊ��')
    %height
    %disp('���θ�Ϊ��')
    %%��������
    disp('��/�߱���rateΪ��')
    rate=width/height
    %MyRegionProps(k).Centroid(1),MyRegionProps(k).Centroid(2)
    %%%�ж���̬�˶�  
    disp('���ľ��룺')
    ddd=sqrt((MyRegionProps(k).Centroid(1)-X).^2+(MyRegionProps(k).Centroid(2)-Y).^2)
    if  ddd<3.5
        %disp('������')
        if  rate<0.7
            disp('վ�����³´�л�ʣ�1.2')
            strtitle = sprintf('վ�����³´�л�ʣ�1.2met');
            
        elseif  rate >0.7
            disp('��չ�ֱۣ��³´�л�ʣ�3.5')
            strtitle = sprintf('��չ�ֱۣ��³´�л�ʣ�3.5met');
        end
    elseif ddd>3.5
        disp('���ߣ��³´�л�ʣ�2.0')
        strtitle = sprintf('���ߣ��³´�л�ʣ�2.0met');
    end
    figure(1);
    imshow(Im);
    title(strtitle);
    rectangle('position',stats(j).BoundingBox,'edgecolor','g');
    
    %���¸�ֵ�����������X Y����
    X=MyRegionProps(k).Centroid(1);
    Y=MyRegionProps(k).Centroid(2);    
    
    
    if i>223
        break
    end
end
msgbox('������ɣ�')
