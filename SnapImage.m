function SnapImage()
imagesPath = '.\\�����ͼ';
if ~exist(imagesPath, 'dir')
    mkdir(imagesPath);
end

[FileName,PathName,FilterIndex] = uiputfile({'*.jpg;*.tif;*.png;*.gif','All Image Files';...
          '*.*','All Files' },'�����ͼ',...
          '.\\�����ͼ\\֤�ݽ�ͼ.jpg');
if isequal(FileName, 0) || isequal(PathName, 0)
    return;
end
fileStr = fullfile(PathName, FileName);
f = getframe(gcf);
f = frame2im(f);
imwrite(f, fileStr);
msgbox('ץͼ�ļ�����ɹ���', '��ʾ��Ϣ');