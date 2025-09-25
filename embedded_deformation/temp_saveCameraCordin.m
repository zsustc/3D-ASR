pointcloud_tmp = rotatePointcloud(pointcloud_new,center_mass,-i*angle);
filename = ['model_tmp',int2str(i),'.stl'];
stlwrite(filename,fout1,pointcloud_tmp,'mode','ascii');
clear pointcloud_tmp;