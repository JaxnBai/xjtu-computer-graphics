bool Rasterizer::inside_triangle(int x, int y, const Vector4f* vertices)
{//参考文献：https://zhuanlan.zhihu.com/p/140907023#:~:text=%E4%B8%89%E8%A7%92%E5%BD%A2%E5%86%85%E6%AF%8F%E4%B8%80%E4%B8%AA%E7%82%B9%E9%83%BD%E6%9C%89%E5%AE%83%E7%9A%84%E9%87%8D%E5%BF%83%E5%9D%90%E6%A0%87%EF%BC%8C%E6%8D%A2%E5%8F%A5%E8%AF%9D%E8%AF%B4%E9%87%8D%E5%BF%83%E5%9D%90%E6%A0%87%E5%85%B6%E5%AE%9E%E6%98%AF%E4%B8%89%E8%A7%92%E5%BD%A2%E5%86%85%E7%9A%84%E7%82%B9%E6%8D%A2%E4%BA%86%E4%B8%80%E4%B8%AA%E9%87%8D%E5%BF%83%E5%9D%90%E6%A0%87%E7%B3%BB%EF%BC%88%E5%8F%82%E8%80%83%E6%A0%87%E9%A2%983%EF%BC%89%E6%9D%A5%E8%A1%A8%E7%A4%BA%E8%AF%A5%E7%82%B9%E7%9A%84%E6%96%B9%E6%B3%95%E3%80%82,%E5%9B%A0%E6%AD%A4%E4%B8%89%E8%A7%92%E5%BD%A2%E7%9A%84%E9%87%8D%E5%BF%83%E4%B9%9F%E6%9C%89%E5%AE%83%E7%9A%84%E9%87%8D%E5%BF%83%E5%9D%90%E6%A0%87%EF%BC%881%2F3%EF%BC%8C1%2F3%EF%BC%8C1%2F3%EF%BC%89%E3%80%82%20%E4%B8%89%E8%A7%92%E5%BD%A2%E7%9A%84%E9%87%8D%E5%BF%83%E5%9D%90%E6%A0%87%E5%B0%86%E8%AF%A5%E4%B8%89%E8%A7%92%E5%BD%A2%E5%88%86%E6%88%90%E4%BA%86%E4%B8%89%E4%B8%AA%E9%9D%A2%E7%A7%AF%E7%9B%B8%E7%AD%89%E7%9A%84%E4%B8%89%E8%A7%92%E5%BD%A2%EF%BC%8C%E6%89%80%E4%BB%A5%E5%9D%90%E6%A0%87%E9%83%BD%E6%98%AF1%2F3%E3%80%82
   Vector3f v[3];
    for (int i = 0; i < 3; i++) v[i] = {vertices[i].x(), vertices[i].y(), 1.0};
    Vector3f bc;
    Vector3f p(float(x), float(y), 1.0f);
    //重心坐标
    bc.x()=((p.y()-v[1].y())*(v[2].x()-v[1].x())-(p.x()-v[1].x())*(v[2].y()-v[1].y()))/((v[0].y()-v[1].y())*(v[2].x()-v[1].x())-(v[0].x()-v[1].x())*(v[2].y()-v[1].y()));
    bc.y()=((p.y()-v[2].y())*(v[0].x()-v[2].x())-(p.x()-v[2].x())*(v[0].y()-v[2].y()))/((v[1].y()-v[2].y())*(v[0].x()-v[2].x())-(v[1].x()-v[2].x())*(v[0].y()-v[2].y()));
    bc.z()=1-bc.x()-bc.y();
    if(bc.x()>0&&bc.x()<1&&bc.y()>0&&bc.y()<1&&bc.z()>0&&bc.z()<1)
        return true;
    
    return false;
}
tuple<float, float, float> Rasterizer::compute_barycentric_2d(float x, float y, const Vector4f* vertices)
{
    float c1 = 0.f, c2 = 0.f, c3 = 0.f;
    Vector3f v[3];
    for (int i = 0; i < 3; i++) v[i] = {vertices[i].x(), vertices[i].y(), 1.0};
    Vector3f p(float(x), float(y), 1.0f);
    c1=((p.y()-v[1].y())*(v[2].x()-v[1].x())-(p.x()-v[1].x())*(v[2].y()-v[1].y()))/((v[0].y()-v[1].y())*(v[2].x()-v[1].x())-(v[0].x()-v[1].x())*(v[2].y()-v[1].y()));
    c2=((p.y()-v[2].y())*(v[0].x()-v[2].x())-(p.x()-v[2].x())*(v[0].y()-v[2].y()))/((v[1].y()-v[2].y())*(v[0].x()-v[2].x())-(v[1].x()-v[2].x())*(v[0].y()-v[2].y()));
    c3=1-c1-c2;
    return {c1, c2, c3};
}
void Rasterizer::draw(const std::vector<Triangle>& TriangleList, const GL::Material& material,
                      const std::list<Light>& lights, const Camera& camera)
{
    // iterate over all triangles in TriangleList
    for (const auto& t : TriangleList) {
        Triangle newtriangle = t;
    
        // transform vertex position to world space for interpolating
        std::array<Vector3f, 3> worldspace_pos;
        for(int i=0;i<3;i++)
        {
            Vector4f a=model*newtriangle.vertex[i];
            worldspace_pos[i](0)=a(0);
            worldspace_pos[i](1)=a(1);
            worldspace_pos[i](2)=a(2);
        }
        // Use vetex_shader to transform vertex attributes(position & normals) to
        // view port and set a new triangle
        VertexShaderPayload tri[3];
        for(int i=0;i<3;i++)
        {
            VertexShaderPayload a={newtriangle.vertex[i],newtriangle.normal[i]};
            tri[i]=vertex_shader(a);//顶点在screen space,法向量在world space
        }
        for(int i=0;i<3;i++)
        {
            newtriangle.vertex[i]=tri[i].position;
           /* Matrix4f a=Matrix4f::Identity();//法向量在screen space坐标系下
            a(0,0)=width/2;
            a(1,1)=height/2;
            a(0,3)=width/2;
            a(1,3)=height/2;
            Vector4f n(newtriangle.normal[i](0),newtriangle.normal[i](1),newtriangle.normal[i](2),1);
            n=((a*Uniforms::MVP).inverse().transpose())*n;//notice
             for(int j=0;j<3;j++)
                newtriangle.normal[i](j)=n(j)/n(3);*/
            newtriangle.normal[i]=tri[i].normal;    
        }
        Triangle tria;
        for(int i=0;i<3;i++)
        {
            tria.vertex[i]=Uniforms::MVP*t.vertex[i];
            tria.normal[i]=tri[i].normal;
        }
        // call rasterize_triangle()
        rasterize_triangle(newtriangle,worldspace_pos,material,lights,camera);
    }
}
void Rasterizer::rasterize_triangle(const Triangle& t, const std::array<Vector3f, 3>& world_pos,
                                    GL::Material material, const std::list<Light>& lights,
                                    Camera camera)
{  
    // discard all pixels out of the range(including x,y,z)
    int min_x = std::max(0, static_cast<int>(std::floor(std::min({t.vertex[0].x(), t.vertex[1].x(), t.vertex[2].x()}))));
    int max_x = std::min(width - 1, static_cast<int>(std::ceil(std::max({t.vertex[0].x(), t.vertex[1].x(), t.vertex[2].x()}))));
    int min_y = std::max(0, static_cast<int>(std::floor(std::min({t.vertex[0].y(), t.vertex[1].y(), t.vertex[2].y()}))));
    int max_y = std::min(height - 1, static_cast<int>(std::ceil(std::max({t.vertex[0].y(), t.vertex[1].y(), t.vertex[2].y()}))));
        
    // if current pixel is in current triange:
    // 1. interpolate depth(use projection correction algorithm)
    // 2. interpolate vertex positon & normal(use function:interpolate())
    // 3. fragment shading(use function:fragment_shader())
    // 4. set pixel
    for (int x = min_x; x <= max_x; ++x) {
        for (int y = min_y; y <= max_y; ++y) {
            // 判断当前像素坐标是否在三角形内
            if (inside_triangle(x, y, t.vertex)) {
                // 计算当前像素在三角形内的重心坐标
                auto [alpha, beta, gamma] = compute_barycentric_2d(static_cast<float>(x), static_cast<float>(y), t.vertex);

                // 插值计算像素的深度值
                Vector4f n0=projection*view*Vector4f(world_pos[0].x(),world_pos[0].y(),world_pos[0].z(),1);
                Vector4f n1=projection*view*Vector4f(world_pos[1].x(),world_pos[1].y(),world_pos[1].z(),1);
                Vector4f n2=projection*view*Vector4f(world_pos[2].x(),world_pos[2].y(),world_pos[2].z(),1);
                float d0=(n0).w();
                float d1=(n1).w();
                float d2=(n2).w();
                float z = 1.0f / (alpha / d0 + beta / d1 + gamma / d2);
                float interpolated_depth=alpha*n0.z()/d0  +beta*n1.z()/d1  +gamma*n2.z()/d2;
                interpolated_depth=interpolated_depth*z;
                // 判断当前像素的深度值是否在可见范围内
                if (interpolated_depth < depth_buf[get_index(x, y)]) {
                    // 插值计算像素的视图空间坐标和法线向量
                    Eigen::Vector3f interpolated_view_pos = interpolate(alpha, beta, gamma, world_pos[0], world_pos[1], world_pos[2], {d0, d1, d2}, interpolated_depth);
                    Eigen::Vector3f interpolated_view_normal = interpolate(alpha, beta, gamma, t.normal[0], t.normal[1], t.normal[2], {d0, d1, d2}, interpolated_depth);

                    // 构造片元着色器需要的Payload
                    FragmentShaderPayload fragment_payload(interpolated_view_pos, interpolated_view_normal.normalized());
                    fragment_payload.world_pos=interpolated_view_pos;
                    // 调用片元着色器进行光照计算
                    Eigen::Vector3f color = fragment_shader(fragment_payload, material, lights, camera);

                    // 将计算得到的颜色存储到帧缓冲中
                    set_pixel({x, y}, color);

                    // 更新深度缓冲
                    depth_buf[get_index(x, y)] = interpolated_depth;
                }
            }
        }
    }        
}
VertexShaderPayload vertex_shader(const VertexShaderPayload& payload)
{
    VertexShaderPayload output_payload = payload;
    
    Vector4f clip= Uniforms::MVP*payload.position;
    for(int i=0;i<4;i++)
        clip(i)=clip(i)/clip(3);
    
    // Vertex position transformation
    Eigen::Matrix4f a=Eigen::Matrix4f::Identity();
    a(0,0)=Uniforms::width/2;
    a(1,1)=Uniforms::height/2;
    a(0,3)=Uniforms::width/2;
    a(1,3)=Uniforms::height/2;
    output_payload.position=a*clip;
    // Viewport transformation
    Vector4f n(payload.normal(0),payload.normal(1),payload.normal(2),1);
    n=Uniforms::inv_trans_M*n;
    for(int i=0;i<3;i++)
    output_payload.normal(i)=n(i)/n(3);
    // Vertex normal transformation
    

    return output_payload;
}

Vector3f phong_fragment_shader(const FragmentShaderPayload& payload, GL::Material material,
                               const std::list<Light>& lights, Camera camera)
{
    //参考文献：https://blog.csdn.net/qq_40297109/article/details/124435570
    //参考文献：https://www.cnblogs.com/lawliet12/p/17719365.html
    //参考文献：https://zhuanlan.zhihu.com/p/452687345

    // ka,kd,ks can be got from material.ambient,material.diffuse,material.specular
    Eigen::Vector3f ka = material.ambient;
    Eigen::Vector3f kd = material.diffuse;
    Eigen::Vector3f ks = material.specular;
    // set ambient light intensity

    // Light Direction
        
    // View Direction
        
    // Half Vector
    Eigen::Vector3f point = payload.world_pos;
    Eigen::Vector3f normal = payload.world_normal;
    Eigen::Vector3f eye_pos= camera.position;
    Eigen::Vector3f amb_light_intensity={1,1,1};
    Eigen::Vector3f result_color = {0, 0, 0};

     Vector3f La ;    // 环境光 
    for(int i=0;i<3;i++)
    {
        La(i)=ka(i)*amb_light_intensity(i);
    }
    result_color += La;
    // Light Attenuation
        
    // Ambient
        
    // Diffuse
        
    // Specular
    for (auto& light : lights)
    {
        // TODO: For each light source in the code, calculate what the *ambient*, *diffuse*, and *specular* 
        // components are. Then, accumulate that result on the *result_color* object.
        Vector3f lightPos = light.position;
        
        float r = (lightPos - point).dot(lightPos - point);

        Vector3f n = normal.normalized();    
        Vector3f l = (lightPos - point).normalized();   
        Vector3f v = (eye_pos - point).normalized();    
        Vector3f h = (v + l).normalized();  

        Vector3f Ld = kd*(light.intensity / r) * std::max(0.0f, n.dot(l));
        Vector3f Ls = ks*(light.intensity / r) * std::pow(std::max(0.0f, n.dot(h)), material.shininess);

        result_color += Ld;
        result_color += Ls;
    }
   // set rendering result max threshold to 255
    for(int i=0;i<3;i++)
    {
        if(result_color(i)<0)result_color(i)=0;
        if(result_color(i)>1)result_color(i)=1;
    }
    printf("result_color:%f %f %f\n",result_color(0),result_color(1),result_color(2));
    return result_color * 255.f;   
}