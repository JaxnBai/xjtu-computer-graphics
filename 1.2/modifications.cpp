Matrix4f Object::model()
{
    const Quaternionf& r = rotation;
    float x,y,z;
    auto [x_angle, y_angle, z_angle] = quaternion_to_ZYX_euler(r.w(), r.x(), r.y(), r.z());
    x=radians(x_angle);
    y=radians(y_angle);
    z=radians(z_angle);
    Matrix4f ms=Matrix4f::Identity(),mrz=Matrix4f::Identity(),mry=Matrix4f::Identity(),mrx=Matrix4f::Identity(),mc=Matrix4f::Identity();
    for(int i=0;i<3;i++)
    {
        ms(i,i)=scaling(i,0);
        mc(i,3)=center(i,0);
        }
    mrz(0,0)=cos(z);mrz(1,1)=mrz(0,0);
    mrz(1,0)=sin(z);mrz(0,1)=-1*mrz(1,0);
    mry(0,0)=cos(y);mry(2,2)=mry(0,0);
    mry(0,2)=sin(y);mry(2,0)=-1*mry(0,2);
    mrx(1,1)=cos(x);mrx(2,2)=mrx(1,1);
    mrx(2,1)=sin(x),mrx(1,2)=-1*mrx(2,1);

    return mc*mrx*mry*mrz*ms;
}