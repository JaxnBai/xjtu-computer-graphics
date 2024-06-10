void Scene::simulation_update()
{
    
    // Calculate the elapsed time since the last update
    start_simulation();
    time_point now = steady_clock::now();
    duration frame_duration = now - last_update;    
    // Convert durations to seconds
    float frame_seconds = duration_cast<duration>(frame_duration).count();
    //frame_seconds=frame_seconds*30;
    //printf("frame_seconds:%f\n",frame_seconds);
    //printf("time_step:%f\n",time_step);
     //getchar();
    float remained_duration = frame_seconds;
   while (remained_duration > time_step)
    {   //int state=1;
    //printf("1\n");
    for (const auto& group : groups) {
        for (const auto& object : group->objects) {
        // Skip objects that are not part of the simulation (if any)
        if (!object) {
            continue;
        }
         object->center += object->velocity * time_step;
        object->velocity += object->force / object->mass * time_step;
           /* if(state)
           {printf("object center:%f %f %f\n",object->center(0),object->center(1),object->center(2));
           printf("object velocity:%f %f %f\n",object->velocity(0),object->velocity(1),object->velocity(2));state=0;}*/
        }}
         remained_duration -= time_step;
    }
    
    last_update +=std::chrono::duration_cast<std::chrono::steady_clock::duration>(std::chrono::duration<double>(frame_seconds-remained_duration)); 
}
void Object::update(vector<Object*>& all_objects)
{
    // 首先调用 step 函数计下一步该物体的运动学状态。
    KineticState current_state{center, velocity, force / mass};
    KineticState next_state = step(prev_state, current_state);
    (void)next_state;
    /*center=next_state.position;
    velocity=next_state.velocity;
    prev_state=current_state;*/
    // 将物体的位置移动到下一步状态处，但暂时不要修改物体的速度。
    // 遍历 all_objects，检查该物体在下一步状态的位置处是否会与其他物体发生碰撞。
    for (auto object : all_objects) {
        (void)object;

        // 检测该物体与另一物体是否碰撞的方法是：
        // 遍历该物体的每一条边，构造与边重合的射线去和另一物体求交，如果求交结果非空、
        // 相交处也在这条边的两个端点之间，那么该物体与另一物体发生碰撞。
        // 请时刻注意：物体 mesh 顶点的坐标都在模型坐标系下，你需要先将其变换到世界坐标系。
        for (size_t i = 0; i < mesh.edges.count(); ++i) {
            array<size_t, 2> v_indices = mesh.edge(i);
            (void)v_indices;
            // v_indices 中是这条边两个端点的索引，以这两个索引为参数调用 GL::Mesh::vertex
            // 方法可以获得它们的坐标，进而用于构造射线。
            if (BVH_for_collision) {
            } else {
            }
            // 根据求交结果，判断该物体与另一物体是否发生了碰撞。
            // 如果发生碰撞，按动量定理计算两个物体碰撞后的速度，并将下一步状态的位置设为
            // current_state.position ，以避免重复碰撞。
        }
    }
    // 将上一步状态赋值为当前状态，并将物体更新到下一步状态。
}
KineticState forward_euler_step([[maybe_unused]] const KineticState& previous,
                                const KineticState& current)
{
    KineticState next_state;
    next_state.position=current.position+current.velocity*time_step;
    next_state.velocity=current.velocity+current.acceleration*time_step;
    next_state.acceleration=current.acceleration;
    return next_state;
    return current;
}