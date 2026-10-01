#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/bool.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2_ros/static_transform_broadcaster.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_listener.h>

#include <opencv2/opencv.hpp>
#include <yaml-cpp/yaml.h>

#include <vector>
#include <queue>
#include <cmath>
#include <string>
#include <memory>
#include <algorithm>
#include <chrono>

using namespace std::chrono_literals;

struct Point2D {
    double x{0.0};
    double y{0.0};
};

struct GridNode {
    int x{0};
    int y{0};
    float g{0.0f};
    float h{0.0f};
    float f{0.0f};
    int parent_idx{-1};

    bool operator>(const GridNode& other) const {
        return f > other.f;
    }
};

class FastNavNode : public rclcpp::Node {
public:
    FastNavNode() : Node("fast_nav_node") {
        tf_buffer_ = std::make_shared<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);
        
        // パラメータ宣言
        this->declare_parameter<std::string>("map_yaml", "");
        this->declare_parameter<double>("max_speed", 0.25);       // 0.25 m/s (250 mm/s)
        this->declare_parameter<double>("accel_xy", 0.4);         // 0.4 m/s^2
        this->declare_parameter<double>("decel_xy", 0.6);         // 0.6 m/s^2
        this->declare_parameter<double>("max_angular", 0.7854);   // 45 deg/s (pi/4 rad/s) - さらに半分
        this->declare_parameter<double>("accel_angular", 1.309);  // 75 deg/s^2
        this->declare_parameter<double>("decel_angular", 1.963);  // 112.5 deg/s^2
        this->declare_parameter<double>("robot_radius", 0.48);    // ロボット衝突判定半径(m) - 実機寸法0.95m×0.95m（半幅0.475m + 余裕0.005m）
        this->declare_parameter<double>("goal_dist_tol", 0.05);   // 到達距離許容差(m) - 50mm
        this->declare_parameter<double>("goal_yaw_tol", 0.087);   // 到達角度許容差(rad, 約5.0度)
        this->declare_parameter<bool>("invert_angular_z", false); // 旋回指令(angular.z)反転フラグ
        this->declare_parameter<double>("yaw_align_timeout", 1.5); // 位置到達後の回頭タイムアウト(秒)

        map_yaml_path_ = this->get_parameter("map_yaml").as_string();
        max_speed_ = this->get_parameter("max_speed").as_double();
        accel_xy_ = this->get_parameter("accel_xy").as_double();
        decel_xy_ = this->get_parameter("decel_xy").as_double();
        max_angular_ = this->get_parameter("max_angular").as_double();
        accel_angular_ = this->get_parameter("accel_angular").as_double();
        decel_angular_ = this->get_parameter("decel_angular").as_double();
        robot_radius_ = this->get_parameter("robot_radius").as_double();
        goal_dist_tol_ = this->get_parameter("goal_dist_tol").as_double();
        goal_yaw_tol_ = this->get_parameter("goal_yaw_tol").as_double();
        invert_angular_z_ = this->get_parameter("invert_angular_z").as_bool();
        yaw_align_timeout_ = this->get_parameter("yaw_align_timeout").as_double();

        // 静的TFブロードキャスター (map -> odom)
        static_tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        broadcastMapToOdomTf();

        // パブリッシャー
        cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("nav_cmd", 10);
        path_pub_ = this->create_publisher<nav_msgs::msg::Path>("plan", 10);
        status_pub_ = this->create_publisher<std_msgs::msg::String>("nav_status", 10);

        // マップおよびコストマップ配信 (QoS: Transient Local で後から接続した RViz / Web UI にも配信)
        auto map_qos = rclcpp::QoS(rclcpp::KeepLast(1)).transient_local().reliable();
        map_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("map", map_qos);
        costmap_pub_ = this->create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", map_qos);

        // サブスクライバー
        odom_sub_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "odom", 10, std::bind(&FastNavNode::odomCallback, this, std::placeholders::_1));
        goal_sub_ = this->create_subscription<geometry_msgs::msg::PoseStamped>(
            "goal_pose", 10, std::bind(&FastNavNode::goalCallback, this, std::placeholders::_1));
        auto_mode_sub_ = this->create_subscription<std_msgs::msg::Bool>(
            "auto_mode", 10, std::bind(&FastNavNode::autoModeCallback, this, std::placeholders::_1));
        scan_sub_ = this->create_subscription<sensor_msgs::msg::LaserScan>(
            "scan", 10, std::bind(&FastNavNode::scanCallback, this, std::placeholders::_1));

        // マップのロード
        if (!map_yaml_path_.empty()) {
            loadMap(map_yaml_path_);
        } else {
            RCLCPP_WARN(this->get_logger(), "No map_yaml parameter provided. Searching default maps...");
            std::string default_map = "src/honrobo_pkg/map/map_red.yaml";
            loadMap(default_map);
        }

        // 100Hz (10ms) 制御タイマー
        control_timer_ = this->create_wall_timer(
            10ms, std::bind(&FastNavNode::controlLoop, this));

        RCLCPP_INFO(this->get_logger(), "🚀 FastNavNode (High Performance C++ Omni Navigator) Initialized at 100Hz!");
    }

private:
    // マップ情報
    int map_w_{0};
    int map_h_{0};
    double resolution_{0.05};
    double origin_x_{-0.65};
    double origin_y_{-3.85};
    std::vector<uint8_t> costmap_;      // 254: 進入不可, 0: 自由
    std::vector<int8_t> occ_grid_data_; // /map 配信用 (-1, 0, 100)

    // ロボット状態
    double cur_x_{0.0};
    double cur_y_{0.0};
    double cur_yaw_{0.0};
    bool odom_received_{false};
    bool auto_mode_{true};

    // 目標・経路状態
    bool has_goal_{false};
    Point2D target_goal_{0.0, 0.0};
    double target_yaw_{0.0};
    std::vector<Point2D> global_path_;
    size_t current_waypoint_idx_{0};

    // 速度状態（台形制御用）
    double cur_speed_xy_{0.0};
    double cur_speed_w_{0.0};
    double prev_cte_{0.0};
    double prev_cur_yaw_{0.0};
    std::chrono::steady_clock::time_point last_control_time_{std::chrono::steady_clock::now()};

    // 到達ラッチ状態
    bool pos_arrived_{false};
    std::chrono::steady_clock::time_point pos_arrived_time_{std::chrono::steady_clock::now()};

    // パラメータ
    std::string map_yaml_path_;
    double max_speed_{0.25};
    double accel_xy_{0.4};
    double decel_xy_{0.6};
    double max_angular_{0.7854};
    double accel_angular_{1.309};
    double decel_angular_{1.963};
    double robot_radius_{0.48};
    double goal_dist_tol_{0.05};
    double goal_yaw_tol_{0.087};
    bool invert_angular_z_{false};
    double yaw_align_timeout_{1.5};

    // ROS 2 通信
    std::shared_ptr<tf2_ros::Buffer> tf_buffer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> static_tf_broadcaster_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_pub_;
    rclcpp::Publisher<nav_msgs::msg::Path>::SharedPtr path_pub_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr status_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr map_pub_;
    rclcpp::Publisher<nav_msgs::msg::OccupancyGrid>::SharedPtr costmap_pub_;
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_sub_;
    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr goal_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr auto_mode_sub_;
    rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan_sub_;
    rclcpp::TimerBase::SharedPtr control_timer_;

    // LiDAR 障害物検知
    bool lidar_obstacle_detected_{false};

    // ── map -> odom 静的TFブロードキャスト ──
    void broadcastMapToOdomTf() {
        geometry_msgs::msg::TransformStamped tf;
        tf.header.stamp = this->now();
        tf.header.frame_id = "map";
        tf.child_frame_id = "odom";
        tf.transform.translation.x = 0.0;
        tf.transform.translation.y = 0.0;
        tf.transform.translation.z = 0.0;
        tf.transform.rotation.x = 0.0;
        tf.transform.rotation.y = 0.0;
        tf.transform.rotation.z = 0.0;
        tf.transform.rotation.w = 1.0;
        static_tf_broadcaster_->sendTransform(tf);
    }

    // ── マップ読み込み & コストマップ生成 ──
    void loadMap(const std::string& yaml_path) {
        try {
            YAML::Node doc = YAML::LoadFile(yaml_path);
            std::string image_rel = doc["image"].as<std::string>();
            resolution_ = doc["resolution"].as<double>();
            origin_x_ = doc["origin"][0].as<double>();
            origin_y_ = doc["origin"][1].as<double>();

            std::string dir = yaml_path.substr(0, yaml_path.find_last_of("/\\"));
            std::string image_path = dir + "/" + image_rel;

            cv::Mat img = cv::imread(image_path, cv::IMREAD_GRAYSCALE);
            if (img.empty()) {
                RCLCPP_ERROR(this->get_logger(), "Failed to load map image: %s", image_path.c_str());
                return;
            }

            map_w_ = img.cols;
            map_h_ = img.rows;

            // 二値化画像 (障害物=0, 通行可能=255)
            cv::Mat bin_img;
            cv::threshold(img, bin_img, 100, 255, cv::THRESH_BINARY);

            // 障害物からのユークリッド距離マップを一瞬で計算 (Distance Transform)
            cv::Mat dist_img;
            cv::distanceTransform(bin_img, dist_img, cv::DIST_L2, 5);

            costmap_.assign(map_w_ * map_h_, 0);
            occ_grid_data_.assign(map_w_ * map_h_, 0);

            for (int r = 0; r < map_h_; ++r) {
                for (int c = 0; c < map_w_; ++c) {
                    int gy = map_h_ - 1 - r; // ROSマップ座標 (左下が0)
                    int gx = c;
                    int idx = gy * map_w_ + gx;

                    // 元画像の占有グリッドデータ
                    uint8_t raw_val = img.at<uint8_t>(r, c);
                    if (raw_val < 50) {
                        occ_grid_data_[idx] = 100; // 障害物
                    } else if (raw_val > 200) {
                        occ_grid_data_[idx] = 0;   // 自由領域
                    } else {
                        occ_grid_data_[idx] = -1;  // 未知
                    }

                    float dist_m = dist_img.at<float>(r, c) * static_cast<float>(resolution_);

                    if (dist_m <= robot_radius_) {
                        costmap_[idx] = 254; // 進入不可 (Lethal)
                    } else if (dist_m <= robot_radius_ + 0.25f) {
                        // コスト勾配 (壁から離れるほど通りやすくする)
                        float ratio = 1.0f - (dist_m - robot_radius_) / 0.25f;
                        costmap_[idx] = static_cast<uint8_t>(ratio * 80.0f);
                    } else {
                        costmap_[idx] = 0;   // 自由領域
                    }
                }
            }

            // /map トピックを配信
            publishMap();

            RCLCPP_INFO(this->get_logger(),
                "✅ Map Loaded: %s (%dx%d, Res: %.3f, Origin: [%.2f, %.2f])",
                image_rel.c_str(), map_w_, map_h_, resolution_, origin_x_, origin_y_);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(this->get_logger(), "Exception in loadMap: %s", e.what());
        }
    }

    void publishMap() {
        nav_msgs::msg::OccupancyGrid map_msg;
        map_msg.header.stamp = this->now();
        map_msg.header.frame_id = "map";
        map_msg.info.resolution = static_cast<float>(resolution_);
        map_msg.info.width = map_w_;
        map_msg.info.height = map_h_;
        map_msg.info.origin.position.x = origin_x_;
        map_msg.info.origin.position.y = origin_y_;
        map_msg.info.origin.position.z = 0.0;
        map_msg.info.origin.orientation.w = 1.0;
        map_msg.data = occ_grid_data_;
        map_pub_->publish(map_msg);

        // コストマップ配信 (障害物膨張および侵入不可領域 254 -> 100)
        nav_msgs::msg::OccupancyGrid costmap_msg = map_msg;
        std::vector<int8_t> cost_data(map_w_ * map_h_, 0);
        for (size_t i = 0; i < costmap_.size(); ++i) {
            if (costmap_[i] >= 254) {
                cost_data[i] = 100;
            } else if (costmap_[i] > 0) {
                cost_data[i] = static_cast<int8_t>(costmap_[i]);
            } else {
                cost_data[i] = (occ_grid_data_[i] < 0) ? -1 : 0;
            }
        }
        costmap_msg.data = cost_data;
        costmap_pub_->publish(costmap_msg);
    }

    // ── 座標変換 ──
    bool worldToGrid(double wx, double wy, int& gx, int& gy) const {
        gx = static_cast<int>(std::round((wx - origin_x_) / resolution_));
        gy = static_cast<int>(std::round((wy - origin_y_) / resolution_));
        return (gx >= 0 && gx < map_w_ && gy >= 0 && gy < map_h_);
    }

    void gridToWorld(int gx, int gy, double& wx, double& wy) const {
        wx = origin_x_ + (gx + 0.5) * resolution_;
        wy = origin_y_ + (gy + 0.5) * resolution_;
    }

    // ── 見通し線チェック (Raycasting Line of Sight) ──
    bool checkLineOfSight(int x0, int y0, int x1, int y1) const {
        int dx = std::abs(x1 - x0);
        int dy = std::abs(y1 - y0);
        int sx = (x0 < x1) ? 1 : -1;
        int sy = (y0 < y1) ? 1 : -1;
        int err = dx - dy;

        int cx = x0;
        int cy = y0;

        while (true) {
            if (cx < 0 || cx >= map_w_ || cy < 0 || cy >= map_h_) return false;
            if (costmap_[cy * map_w_ + cx] >= 254) return false;

            if (cx == x1 && cy == y1) break;

            int e2 = 2 * err;
            if (e2 > -dy) {
                err -= dy;
                cx += sx;
            }
            if (e2 < dx) {
                err += dx;
                cy += sy;
            }
        }
        return true;
    }

    // ── A* 経路探索 ──
    std::vector<Point2D> planAStar(double start_x, double start_y, double goal_x, double goal_y) {
        std::vector<Point2D> path;
        int sgx, sgy, ggx, ggy;
        if (!worldToGrid(start_x, start_y, sgx, sgy)) {
            RCLCPP_WARN(this->get_logger(), "Start position out of map bounds!");
            return path;
        }
        if (!worldToGrid(goal_x, goal_y, ggx, ggy)) {
            RCLCPP_WARN(this->get_logger(), "Goal position out of map bounds!");
            return path;
        }

        // ゴールが進入不可領域にある場合、近傍の通行可能セルを探索
        if (costmap_[ggy * map_w_ + ggx] >= 254) {
            RCLCPP_WARN(this->get_logger(), "Goal is in collision cost. Finding nearest free cell...");
            bool found_free = false;
            for (int r = 1; r <= 25 && !found_free; ++r) {
                for (int dy = -r; dy <= r && !found_free; ++dy) {
                    for (int dx = -r; dx <= r && !found_free; ++dx) {
                        int nx = ggx + dx;
                        int ny = ggy + dy;
                        if (nx >= 0 && nx < map_w_ && ny >= 0 && ny < map_h_) {
                            if (costmap_[ny * map_w_ + nx] < 254) {
                                ggx = nx;
                                ggy = ny;
                                found_free = true;
                            }
                        }
                    }
                }
            }
            if (!found_free) {
                RCLCPP_ERROR(this->get_logger(), "Could not find free cell near goal!");
                return path;
            }
        }

        auto start_time = std::chrono::high_resolution_clock::now();

        std::priority_queue<GridNode, std::vector<GridNode>, std::greater<GridNode>> open_set;
        std::vector<float> g_scores(map_w_ * map_h_, 1e9f);
        std::vector<int> parent_map(map_w_ * map_h_, -1);
        std::vector<bool> closed_set(map_w_ * map_h_, false);

        int start_idx = sgy * map_w_ + sgx;
        int goal_idx = ggy * map_w_ + ggx;

        g_scores[start_idx] = 0.0f;
        open_set.push({sgx, sgy, 0.0f, static_cast<float>(std::hypot(ggx - sgx, ggy - sgy)), 0.0f, -1});

        const int dx_dir[8] = {1, -1, 0, 0, 1, -1, 1, -1};
        const int dy_dir[8] = {0, 0, 1, -1, 1, 1, -1, -1};
        const float move_cost[8] = {1.0f, 1.0f, 1.0f, 1.0f, 1.4142f, 1.4142f, 1.4142f, 1.4142f};

        bool found_path = false;

        while (!open_set.empty()) {
            GridNode current = open_set.top();
            open_set.pop();

            int curr_idx = current.y * map_w_ + current.x;
            if (closed_set[curr_idx]) continue;
            closed_set[curr_idx] = true;

            if (current.x == ggx && current.y == ggy) {
                found_path = true;
                break;
            }

            for (int i = 0; i < 8; ++i) {
                int nx = current.x + dx_dir[i];
                int ny = current.y + dy_dir[i];

                if (nx < 0 || nx >= map_w_ || ny < 0 || ny >= map_h_) continue;
                int n_idx = ny * map_w_ + nx;

                if (closed_set[n_idx]) continue;
                if (costmap_[n_idx] >= 254) continue;

                // コスト加算 (壁から離れているほど通りやすい)
                float step_c = move_cost[i] + (costmap_[n_idx] / 50.0f);
                float tentative_g = g_scores[curr_idx] + step_c;

                if (tentative_g < g_scores[n_idx]) {
                    g_scores[n_idx] = tentative_g;
                    parent_map[n_idx] = curr_idx;
                    float h = static_cast<float>(std::hypot(ggx - nx, ggy - ny));
                    open_set.push({nx, ny, tentative_g, h, tentative_g + h, curr_idx});
                }
            }
        }

        auto end_time = std::chrono::high_resolution_clock::now();
        double search_time_ms = std::chrono::duration<double, std::milli>(end_time - start_time).count();

        if (!found_path) {
            RCLCPP_WARN(this->get_logger(), "A* failed to find path in %.2f ms", search_time_ms);
            return path;
        }

        // 経路の復元
        std::vector<std::pair<int, int>> raw_grid_path;
        int curr = goal_idx;
        while (curr != -1) {
            int cx = curr % map_w_;
            int cy = curr / map_w_;
            raw_grid_path.push_back({cx, cy});
            if (curr == start_idx) break;
            curr = parent_map[curr];
        }
        std::reverse(raw_grid_path.begin(), raw_grid_path.end());

        // ── 見通し線ショートカット (Raycasting Path Pruning) ──
        std::vector<std::pair<int, int>> pruned_grid_path;
        pruned_grid_path.push_back(raw_grid_path.front());

        size_t current_idx = 0;
        while (current_idx < raw_grid_path.size() - 1) {
            size_t furthest = current_idx + 1;
            for (size_t next_idx = raw_grid_path.size() - 1; next_idx > current_idx; --next_idx) {
                if (checkLineOfSight(raw_grid_path[current_idx].first, raw_grid_path[current_idx].second,
                                     raw_grid_path[next_idx].first, raw_grid_path[next_idx].second)) {
                    furthest = next_idx;
                    break;
                }
            }
            pruned_grid_path.push_back(raw_grid_path[furthest]);
            current_idx = furthest;
        }

        // ワールド座標へ変換
        path.push_back({start_x, start_y}); // 正確な現在地
        for (size_t i = 1; i < pruned_grid_path.size() - 1; ++i) {
            double wx, wy;
            gridToWorld(pruned_grid_path[i].first, pruned_grid_path[i].second, wx, wy);
            path.push_back({wx, wy});
        }
        path.push_back({goal_x, goal_y});   // 正確な目標地点

        // 経路線分を密に補間 (8cm刻み) して滑らかな追従と正確な横偏差計算を実現
        std::vector<Point2D> dense_path;
        if (!path.empty()) {
            dense_path.push_back(path.front());
            for (size_t i = 0; i < path.size() - 1; ++i) {
                const auto& p1 = path[i];
                const auto& p2 = path[i+1];
                double seg_len = std::hypot(p2.x - p1.x, p2.y - p1.y);
                int steps = std::max(1, static_cast<int>(std::ceil(seg_len / 0.08)));
                for (int s = 1; s <= steps; ++s) {
                    double r = static_cast<double>(s) / steps;
                    dense_path.push_back({p1.x + r * (p2.x - p1.x), p1.y + r * (p2.y - p1.y)});
                }
            }
        } else {
            dense_path = path;
        }

        RCLCPP_INFO(this->get_logger(),
            "✨ A* Path Found in %.2f ms | Raw: %zu pts -> Pruned: %zu pts -> Dense: %zu pts",
            search_time_ms, raw_grid_path.size(), path.size(), dense_path.size());

        publishPath(dense_path);
        return dense_path;
    }

    void publishPath(const std::vector<Point2D>& path) {
        nav_msgs::msg::Path path_msg;
        path_msg.header.stamp = this->now();
        path_msg.header.frame_id = "map";

        for (const auto& pt : path) {
            geometry_msgs::msg::PoseStamped pose;
            pose.header = path_msg.header;
            pose.pose.position.x = pt.x;
            pose.pose.position.y = pt.y;
            pose.pose.orientation.w = 1.0;
            path_msg.poses.push_back(pose);
        }
        path_pub_->publish(path_msg);
    }

    // ── オドメトリコールバック (TFによるAMCL補正対応) ──
    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        if (!tf_buffer_) return;
        try {
            // AMCL等の map -> odom が存在する場合は、完全な自己位置 (map -> base_link) を取得
            auto t = tf_buffer_->lookupTransform("map", "base_link", tf2::TimePointZero);
            cur_x_ = t.transform.translation.x;
            cur_y_ = t.transform.translation.y;
            tf2::Quaternion q(
                t.transform.rotation.x, t.transform.rotation.y,
                t.transform.rotation.z, t.transform.rotation.w);
            tf2::Matrix3x3 m(q);
            double r, p, y;
            m.getRPY(r, p, y);
            cur_yaw_ = y;
            odom_received_ = true;
        } catch (const tf2::TransformException & ex) {
            // AMCLが起動していない場合は、生オドメトリを直接マップ座標としてフォールバック使用
            cur_x_ = msg->pose.pose.position.x;
            cur_y_ = msg->pose.pose.position.y;

            const auto& q_msg = msg->pose.pose.orientation;
            tf2::Quaternion tf_q(q_msg.x, q_msg.y, q_msg.z, q_msg.w);
            tf2::Matrix3x3 m_msg(tf_q);
            double roll, pitch, yaw;
            m_msg.getRPY(roll, pitch, yaw);
            cur_yaw_ = yaw;

            odom_received_ = true;
        }
    }

    // ── 目標地点受信コールバック ──
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
        if (!odom_received_) {
            RCLCPP_WARN(this->get_logger(), "Cannot navigate: /odom not received yet.");
            return;
        }

        target_goal_.x = msg->pose.position.x;
        target_goal_.y = msg->pose.position.y;

        const auto& q = msg->pose.orientation;
        double norm_sq = q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w;
        if (norm_sq > 0.01) {
            tf2::Quaternion tf_q(q.x, q.y, q.z, q.w);
            tf2::Matrix3x3 m(tf_q);
            double r, p, yaw;
            m.getRPY(r, p, yaw);
            target_yaw_ = std::isnan(yaw) ? cur_yaw_ : yaw;
        } else {
            // クォータニオン未初期化時は現在の向きを維持
            target_yaw_ = cur_yaw_;
        }

        RCLCPP_INFO(this->get_logger(), "🎯 New Goal Received: (%.3f, %.3f), Yaw: %.1f deg",
                    target_goal_.x, target_goal_.y, target_yaw_ * 180.0 / M_PI);

        global_path_ = planAStar(cur_x_, cur_y_, target_goal_.x, target_goal_.y);
        if (!global_path_.empty()) {
            has_goal_ = true;
            pos_arrived_ = false;
            prev_cur_yaw_ = cur_yaw_;
            current_waypoint_idx_ = 0;
            cur_speed_xy_ = 0.0; // 加速初期化
            prev_cte_ = 0.0;     // 横偏差初期化
            publishStatus("moving");
        } else {
            has_goal_ = false;
            pos_arrived_ = false;
            publishStatus("planning_failed");
        }
    }

    // ── 自動運転モードコールバック ──
    void autoModeCallback(const std_msgs::msg::Bool::SharedPtr msg) {
        auto_mode_ = msg->data;
        if (!auto_mode_ && has_goal_) {
            RCLCPP_WARN(this->get_logger(), "Auto mode canceled by user. Stopping.");
            has_goal_ = false;
            pos_arrived_ = false;
            stopRobot();
            publishStatus("cancelled");
        }
    }

    // ── LiDAR スキャンコールバック ──
    void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr msg) {
        if (!has_goal_) return;

        bool obstacle_near = false;
        float angle = msg->angle_min;
        for (size_t i = 0; i < msg->ranges.size(); ++i, angle += msg->angle_increment) {
            float r = msg->ranges[i];
            if (r > msg->range_min && r < 0.65f) {
                if (std::abs(angle) < 0.785f) {
                    obstacle_near = true;
                    break;
                }
            }
        }
        lidar_obstacle_detected_ = obstacle_near;
    }

    // ── 角度正規化 (-pi ~ pi) ──
    static double normalizeAngle(double angle) {
        while (angle > M_PI) angle -= 2.0 * M_PI;
        while (angle < -M_PI) angle += 2.0 * M_PI;
        return angle;
    }

    // ── 100Hz 制御メインループ (Pure Pursuit + 33kg 台形速度制御) ──
    void controlLoop() {
        if (!has_goal_ || !auto_mode_ || global_path_.empty()) {
            return;
        }

        auto now = std::chrono::steady_clock::now();
        double dt = std::chrono::duration<double>(now - last_control_time_).count();
        last_control_time_ = now;
        if (dt <= 0.0 || dt > 0.1) dt = 0.01;

        // 1. ゴールまでの直線距離と角度誤差
        double dx_goal = target_goal_.x - cur_x_;
        double dy_goal = target_goal_.y - cur_y_;
        double dist_to_goal = std::hypot(dx_goal, dy_goal);
        double yaw_error = normalizeAngle(target_yaw_ - cur_yaw_);

        // 2. ゴール到達判定
        if (dist_to_goal <= goal_dist_tol_ && std::abs(yaw_error) <= goal_yaw_tol_) {
            RCLCPP_INFO(this->get_logger(), "🏁 Goal Reached Successfully! DistErr=%.1f mm, YawErr=%.1f deg",
                        dist_to_goal * 1000.0, yaw_error * 180.0 / M_PI);
            stopRobot();
            has_goal_ = false;
            publishStatus("arrived");
            return;
        }

        // デバッグ用: 残り0.2m以下になったら0.5秒おきにログ出力
        if (dist_to_goal < 0.2) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 500,
                "[Nav Debug] Dist: %.3f m, YawErr: %.1f deg, CurSpdXY: %.3f, PosArrived: %d",
                dist_to_goal, yaw_error * 180.0 / M_PI, cur_speed_xy_, (int)pos_arrived_);
        }

        // 3. 最も近いウェイポイントの探索 (前方25点以内から最近傍を検索)
        size_t search_end = std::min(global_path_.size(), current_waypoint_idx_ + 25);
        size_t closest_idx = current_waypoint_idx_;
        double min_dist_to_path = 1e9;
        for (size_t i = current_waypoint_idx_; i < search_end; ++i) {
            double d = std::hypot(global_path_[i].x - cur_x_, global_path_[i].y - cur_y_);
            if (d < min_dist_to_path) {
                min_dist_to_path = d;
                closest_idx = i;
            }
        }
        current_waypoint_idx_ = closest_idx;

        // 4. 前方注視点 (Lookahead Point) の探索 (前方約 0.20m 〜 0.30m)
        const double lookahead_dist = 0.25;
        size_t lookahead_idx = current_waypoint_idx_;
        double acc_d = 0.0;
        for (size_t i = current_waypoint_idx_; i < global_path_.size() - 1; ++i) {
            acc_d += std::hypot(global_path_[i+1].x - global_path_[i].x,
                                global_path_[i+1].y - global_path_[i].y);
            lookahead_idx = i + 1;
            if (acc_d >= lookahead_dist) {
                break;
            }
        }
        Point2D target_pt = global_path_[lookahead_idx];

        // 5. 接線ベクトル (Tangent: 進捗方向) と 法線ベクトル (Normal: 横偏差方向) の算出
        Point2D p_curr_path = global_path_[current_waypoint_idx_];
        double tan_dx = target_pt.x - p_curr_path.x;
        double tan_dy = target_pt.y - p_curr_path.y;
        double tan_len = std::hypot(tan_dx, tan_dy);
        if (tan_len < 1e-4) {
            tan_dx = target_goal_.x - cur_x_;
            tan_dy = target_goal_.y - cur_y_;
            tan_len = std::hypot(tan_dx, tan_dy);
        }
        double t_x = (tan_len > 1e-4) ? (tan_dx / tan_len) : 1.0;
        double t_y = (tan_len > 1e-4) ? (tan_dy / tan_len) : 0.0;
        double n_x = -t_y;  // 左向き法線 (+CTE 方向)
        double n_y =  t_x;

        // 6. 横偏差 (Cross-Track Error: CTE) の算出
        // パス最近傍点からロボット現在位置への相対ベクトル
        double rel_x = cur_x_ - p_curr_path.x;
        double rel_y = cur_y_ - p_curr_path.y;
        double cte = rel_x * n_x + rel_y * n_y; // +: 左に逸脱, -: 右に逸脱

        // 横偏差PD補正速度 (オムニホイールの真骨頂: 向きを変えずに即座に線上に引き戻す)
        double cte_d = (cte - prev_cte_) / dt;
        prev_cte_ = cte;
        double kp_cte = 2.5;   // Pゲイン (1/s)
        double kd_cte = 0.08;  // Dゲイン (s)
        double v_perp = -(kp_cte * cte + kd_cte * cte_d);
        double max_perp = std::min(0.18, max_speed_ * 0.75);
        v_perp = std::clamp(v_perp, -max_perp, max_perp);

        // 7. 残り全経路距離の積算と台形速度プロファイル
        // 現在の目標点までの距離をベースとする
        double total_rem_dist = dist_to_goal;
        // ゴール直近 (0.3m以内) になったら経路の残り長さを加算しない（オーバーシュート防止）
        if (dist_to_goal > 0.3) {
            for (size_t i = lookahead_idx; i < global_path_.size() - 1; ++i) {
                total_rem_dist += std::hypot(global_path_[i+1].x - global_path_[i].x,
                                             global_path_[i+1].y - global_path_[i].y);
            }
        }

        double v_allow = std::sqrt(2.0 * decel_xy_ * std::max(0.0, total_rem_dist - 0.01));
        double target_v = std::min(max_speed_, v_allow);

        // LiDAR近接障害物がある場合は徐行
        if (lidar_obstacle_detected_) {
            target_v = std::min(target_v, 0.12);
        }

        // スルーレート（加速度制限）
        if (target_v > cur_speed_xy_) {
            cur_speed_xy_ = std::min(target_v, cur_speed_xy_ + accel_xy_ * dt);
        } else {
            cur_speed_xy_ = std::max(target_v, cur_speed_xy_ - decel_xy_ * dt);
        }

        // 最低速度保証 (停止寸前で止まりきらない現象の防止)
        if (total_rem_dist > goal_dist_tol_ && cur_speed_xy_ < 0.04) {
            cur_speed_xy_ = 0.04;
        }

        // 8. フィールド速度合成 (進捗方向ベクトル + 横偏差補正ベクトル)
        // 偏差が大きい時は直進速度を抑制し、軌道復帰を最優先する
        double cte_atten = std::clamp(1.0 - 2.5 * std::abs(cte), 0.35, 1.0);
        double v_parallel = cur_speed_xy_ * cte_atten;
        double vx_field = v_parallel * t_x + v_perp * n_x;
        double vy_field = v_parallel * t_y + v_perp * n_y;

        // 到達直前 (目標許容差以内) のタイマー開始処理
        if (dist_to_goal <= goal_dist_tol_) {
            if (!pos_arrived_) {
                pos_arrived_ = true;
                pos_arrived_time_ = now;
                RCLCPP_INFO(this->get_logger(), "Entered goal tolerance (%.3f m). Starting timeout.", dist_to_goal);
            }
            // 目標付近での旋回中の位置ズレを防ぐため、並進速度を微速(3cm/s)に制限して静かに中心へ寄せる
            cur_speed_xy_ = std::min(cur_speed_xy_, 0.03);
            vx_field = std::clamp(vx_field, -0.03, 0.03);
            vy_field = std::clamp(vy_field, -0.03, 0.03);
        }

        // タイムアウト監視 (一度でも目標距離に入ったらタイマーはリセットしない)
        if (pos_arrived_) {
            double elapsed = std::chrono::duration<double>(now - pos_arrived_time_).count();
            // 180度旋回には3秒以上かかるため、タイムアウトを長めに設定（5.0秒）
            if (elapsed > 5.0) {
                RCLCPP_WARN(this->get_logger(), "Nav timeout (%.2fs)! Dist=%.3f, YawErr=%.1f deg. Stopping.", 
                            elapsed, dist_to_goal, yaw_error * 180.0 / M_PI);
                stopRobot();
                has_goal_ = false;
                publishStatus("arrived");
                return;
            }
        }



        // 本ロボットのSTM32 CANプロトコル (0x510):
        //   linear.y (VY) = 前進(+) / 後進(-)
        //   linear.x (VX) = 左移動(+) / 右移動(-)
        double v_forward =  vx_field * std::cos(cur_yaw_) + vy_field * std::sin(cur_yaw_);
        double v_lateral = -vx_field * std::sin(cur_yaw_) + vy_field * std::cos(cur_yaw_);

        // 8. 旋回速度制御 (Yaw)
        double target_w = std::clamp(1.5 * yaw_error, -max_angular_, max_angular_);
        if (invert_angular_z_) {
            target_w = -target_w;
        }
        if (std::abs(yaw_error) <= goal_yaw_tol_) {
            target_w = 0.0;
        }

        // 旋回加減速スルーレート
        if (target_w > cur_speed_w_) {
            cur_speed_w_ = std::min(target_w, cur_speed_w_ + accel_angular_ * dt);
        } else {
            cur_speed_w_ = std::max(target_w, cur_speed_w_ - decel_angular_ * dt);
        }

        // 4輪オムニ車輪最大速度の飽和防止 (並進速度 + 旋回周速度 <= max_speed_)
        double robot_wheel_r = 0.35; // 中心からホイールまでの実効半径(m)
        double rot_lin_v = std::abs(cur_speed_w_) * robot_wheel_r;
        double trans_v = std::hypot(v_lateral, v_forward);
        double max_comb = trans_v + rot_lin_v;
        if (max_comb > max_speed_ && max_comb > 1e-4) {
            double scale = max_speed_ / max_comb;
            v_lateral *= scale;
            v_forward *= scale;
            cur_speed_w_ *= scale;
        }

        // 9. 速度指令パブリッシュ (/nav_cmd)
        geometry_msgs::msg::Twist cmd;
        cmd.linear.x = v_lateral;
        cmd.linear.y = v_forward;
        cmd.angular.z = cur_speed_w_;
        cmd_pub_->publish(cmd);
    }

    void stopRobot() {
        geometry_msgs::msg::Twist cmd;
        cmd_pub_->publish(cmd);
        cur_speed_xy_ = 0.0;
        cur_speed_w_ = 0.0;
    }

    void publishStatus(const std::string& status) {
        std_msgs::msg::String msg;
        msg.data = status;
        status_pub_->publish(msg);
    }
};

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<FastNavNode>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
