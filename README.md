# cartographer_global_relocation
实现cartographer重定位功能，目前已经测试能够实现在6米180度范围内的全局重定位。


#  1.简介

 本博客主要介绍cartographer全局重定位的实现，目前已经测试能够实现在6米180度范围内的全局重定位，原则上应该支持100平空间的全局重定位。 

## 核心流程

这个重定位函数的流程，然后逐个部分给出实现代码。
```
1.创建线程池；
2.点云过一次体素滤波；
3.为每一个 Submap 创建 FastCorrelativeScanMatcher 匹配器，在线程池中执行，并等待所有匹配器创建完成；
4.遍历每一个匹配器，调用 MatchFullSubmap 方法，记录匹配结果和分数，在线程池中执行，并等待所有匹配完成；
5.找出匹配得分最高的位姿，创建 CeresScanMatcher2D 匹配器再次匹配得到准确的位姿。
```
实现代码如下：

```javascript
bool PoseGraph2D::PerformGlobalLocalization(cartographer::sensor::PointCloud laser_point_cloud,float cutoff, transform::Rigid2d* best_pose_estimate, float* best_score)
{
    auto my_thread_pool = std::make_unique<common::ThreadPool>(std::thread::hardware_concurrency());
    assert(my_thread_pool != nullptr);
    LOG(INFO) << "laser_point_cloud.points_.size()1 : " << laser_point_cloud.points().size();
    const cartographer::sensor::PointCloud filtered_point_cloud = cartographer::sensor::VoxelFilter(laser_point_cloud, 0.05);  
    // const cartographer::sensor::PointCloud filtered_point_cloud = match_point_cloud_;
    if (filtered_point_cloud.empty()) {
        LOG(ERROR) << "Filtered point cloud is empty!";
        return false;
    }
    LOG(INFO) << "filtered_point_cloud.points_.size() : " << filtered_point_cloud.points().size();
    LOG(INFO) << "cutoff : " << cutoff;
    int32_t submap_size = static_cast<int>(data_.submap_data.size());
    absl::BlockingCounter created_counter{submap_size};
    std::vector<std::shared_ptr<scan_matching::FastCorrelativeScanMatcher2D>> matchers(submap_size);
    std::vector<const cartographer::mapping::Grid2D*> submaps(submap_size);
    LOG(INFO) << "Submap size: " << submap_size;
    size_t index = 0;
    for (const auto& submap_id_data : data_.submap_data) 
    {
        if (submap_id_data.id.trajectory_id != 0) {
            created_counter.DecrementCount();
            continue;
        }
        auto task = absl::make_unique<common::Task>();
        task->SetWorkItem([this, &matchers, &created_counter, index, submap_id = submap_id_data.id, &submaps] {
            try {
                const auto& submap_data = data_.submap_data.at(submap_id);
                if (!submap_data.submap) {
                    LOG(ERROR) << "Submap is null for index " << index;
                    throw std::runtime_error("Submap is null");
                }
                submaps[index] = static_cast<const Submap2D*>(submap_data.submap.get())->grid();
                matchers[index] = std::make_unique<scan_matching::FastCorrelativeScanMatcher2D>(
                    *submaps[index],
                    options_.constraint_builder_options().fast_correlative_scan_matcher_options());
                LOG(INFO) << "Task completed for index: " << index;
            } catch (const std::exception& e) {
                LOG(ERROR) << "Error in task for index " << index << ": " << e.what();
            }
            created_counter.DecrementCount();
        });

        my_thread_pool->Schedule(std::move(task));
        index++;
    }

    LOG(INFO) << "Total submaps processed: " << index;
    created_counter.Wait();
    LOG(INFO) << "PoseGraph2D::PerformGlobalLocalization 728.";
    size_t matcher_size = index;
    std::vector<float> score_set(matcher_size, -std::numeric_limits<float>::infinity());
    std::vector<transform::Rigid2d> pose_set(matcher_size);    
    absl::BlockingCounter matched_counter{matcher_size};
    std::atomic_bool has_matched{false};    
    for (size_t i = 0; i < matcher_size; i++) 
    {
        auto task = absl::make_unique<common::Task>();
        task->SetWorkItem([i, &filtered_point_cloud, &matchers, &score_set, &pose_set, cutoff, &matched_counter, &has_matched] {
            if (!matchers[i]) 
            {
                LOG(ERROR) << "Matcher is null at index " << i;
                matched_counter.DecrementCount();
                return;
            }
            float score = -1;
            transform::Rigid2d pose_estimate = transform::Rigid2d::Identity();
            LOG(INFO) << "Processing2 matcher index: " << i;
            try {
                if (matchers[i]->MatchFullSubmap(filtered_point_cloud, cutoff, &score, &pose_estimate)) 
                {
                    score_set[i] = score;
                    pose_set[i] = pose_estimate;
                    has_matched = true;
                } else {
                  LOG(INFO) << "match failed. ";
                }
            } catch (const std::exception& e) {
                LOG(ERROR) << "Exception in MatchFullSubmap at index " << i << ": " << e.what();
            }

            matched_counter.DecrementCount();
        });
        my_thread_pool->Schedule(std::move(task));
    }

    matched_counter.Wait();

    if (!has_matched) 
    {
        LOG(ERROR) << "No matches found!";
        return false;
    }


    int max_position = std::distance(score_set.begin(), std::max_element(score_set.begin(), score_set.end()));
    *best_score = score_set[max_position];
    *best_pose_estimate = pose_set[max_position];

    auto csm = std::make_unique<scan_matching::CeresScanMatcher2D>(
        options_.constraint_builder_options().ceres_scan_matcher_options());

    ceres::Solver::Summary unused_summary;

    try {
        csm->Match(best_pose_estimate->translation(), *best_pose_estimate,
                   filtered_point_cloud, *submaps[max_position],
                   best_pose_estimate, &unused_summary);
    } catch (const std::exception& e) {
        LOG(ERROR) << "CeresScanMatcher2D failed: " << e.what();
        return false;
    }

    LOG(INFO) << "PoseGraph2D::PerformGlobalLocalization end.";
    return true;
}
