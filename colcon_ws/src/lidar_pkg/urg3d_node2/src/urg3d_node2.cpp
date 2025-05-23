// Copyright 2022 HOKUYO AUTOMATIC CO., LTD.
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "urg3d_node2/urg3d_node2.hpp"
#include "urg3d_node2/auxiliary_define.hpp"

namespace urg3d_node2
{

Urg3dNode2::Urg3dNode2(const rclcpp::NodeOptions & node_options)
: rclcpp_lifecycle::LifecycleNode("urg3d_node2", node_options),
  error_count_(0),
  is_connected_(false),
  is_measurement_started_(false),
  is_stable_(false),
  use_intensity_(false),
  system_latency_(0ns),
  user_latency_(0ns)
{
  // urg_open後にLiDARの電源がOFFになった状態でLiDARと通信しようとするとSIGPIPEシグナルが発生する
  // ROS1ではROSのライブラリで設定されていたがROS2では未対応のため、ここで設定する
  std::signal(SIGPIPE, SIG_IGN);
  
  // パラメータの登録
  ip_address_ = declare_parameter<std::string>("ip_address", "192.168.0.10");
  ip_port_ = declare_parameter<int>("ip_port", 10940);
  frame_id_ = declare_parameter<std::string>("frame_id", "hokuyo3d");
  range_min_ = declare_parameter<double>("range_min", 0.1);
  interlace_h_ = declare_parameter<int>("interlace_h", 1);
  interlace_v_ = declare_parameter<int>("interlace_v", 1);
  output_cycle_ = declare_parameter<std::string>("output_cycle", "frame");
  calibrate_time_ = declare_parameter<bool>("calibrate_time", false);
  synchronize_time_ = declare_parameter<bool>("synchronize_time", false);
  publish_intensity_ = declare_parameter<bool>("publish_intensity", true);
  publish_auxiliary_ = declare_parameter<bool>("publish_auxiliary", false);
  error_limit_ = declare_parameter<int>("error_limit", 4);
  error_reset_period_ = declare_parameter<double>("error_reset_period", 10.0),
  diagnostics_tolerance_ = declare_parameter<double>("diagnostics_tolerance", 0.05);
  diagnostics_window_time_ = declare_parameter<double>("diagnostics_window_time", 5.0);
  time_offset_ = declare_parameter<double>("time_offset", 0.0);
}

// デストラクタ
Urg3dNode2::~Urg3dNode2()
{
  // スレッドの停止
  stop_thread();
}

// onConfigure
Urg3dNode2::CallbackReturn Urg3dNode2::on_configure(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "transition Configuring from %s", state.label().c_str());

    initialize();

    if(!connect()) {
        return CallbackReturn::FAILURE;
    }

    // Publisher設定
    scan_pub_2 = create_publisher<sensor_msgs::msg::PointCloud2>("hokuyo_cloud2", rclcpp::QoS(20));
    imu_pub_ = create_publisher<sensor_msgs::msg::Imu>("imu", rclcpp::QoS(20));
    mag_pub_ = create_publisher<sensor_msgs::msg::MagneticField>("mag", rclcpp::QoS(20));
    temp_pub_ = create_publisher<sensor_msgs::msg::Temperature>("temp", rclcpp::QoS(20));

    // スレッド起動
    start_thread();

    return CallbackReturn::SUCCESS;
}

// onActivate
Urg3dNode2::CallbackReturn Urg3dNode2::on_activate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "transition Activating from %s", state.label().c_str());

    if(!is_connected_) {
        return CallbackReturn::ERROR;
    }
    else{
        // publisherの有効化
        if(scan_pub_2){
            scan_pub_2->on_activate();
        }
        if(imu_pub_){
            imu_pub_->on_activate();
        }
        if(mag_pub_){
            mag_pub_->on_activate();
        }
        if(temp_pub_){
            temp_pub_->on_activate();
        }

        // Diagnostics開始
        start_diagnostics();

        // 累計エラーカウントの初期化
        total_error_count_ = 0;
    }

    return CallbackReturn::SUCCESS;
}

// onDeactivate
Urg3dNode2::CallbackReturn Urg3dNode2::on_deactivate(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "transition Deactivating from %s", state.label().c_str());

    // Diagnostics停止
    stop_diagnostics();

    if(!is_connected_){
        return CallbackReturn::ERROR;
    }

    return CallbackReturn::SUCCESS;
}

// onCleanup
Urg3dNode2::CallbackReturn Urg3dNode2::on_cleanup(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "transition CleaningUp from %s", state.label().c_str());

    // スレッド停止
    stop_thread();

    // publisherの解放
    scan_pub_2.reset();
    imu_pub_.reset();
    mag_pub_.reset();
    temp_pub_.reset();

    // 切断
    disconnect();

    return CallbackReturn::SUCCESS;
}

// onShutdown
Urg3dNode2::CallbackReturn Urg3dNode2::on_shutdown(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "transition Shutdown from %s", state.label().c_str());

    // スレッド停止
    stop_thread();

    // Diagnostics停止
    stop_diagnostics();

    // publisherの解放
    scan_pub_2.reset();
    imu_pub_.reset();
    mag_pub_.reset();
    temp_pub_.reset();

    // 切断
    disconnect();

    return CallbackReturn::SUCCESS;
}

// onError
Urg3dNode2::CallbackReturn Urg3dNode2::on_error(const rclcpp_lifecycle::State & state)
{
    RCLCPP_DEBUG(get_logger(), "transition Error from %s", state.label().c_str());

    // スレッドの停止
    stop_thread();

    // Diagnostics停止
    stop_diagnostics();

    // publisherの解放
    scan_pub_2.reset();
    imu_pub_.reset();
    mag_pub_.reset();
    temp_pub_.reset();

    // 切断
    disconnect();

    return CallbackReturn::SUCCESS;
}

// 初期化
void Urg3dNode2::initialize()
{
    // パラメータ取得
    ip_address_ = get_parameter("ip_address").as_string();
    ip_port_ = get_parameter("ip_port").as_int();
    frame_id_ = get_parameter("frame_id").as_string();
    range_min_ = get_parameter("range_min").as_double();
    interlace_h_= get_parameter("interlace_h").as_int();
    interlace_v_ = get_parameter("interlace_v").as_int();
    output_cycle_ = get_parameter("output_cycle").as_string();
    publish_intensity_ = get_parameter("publish_intensity").as_bool();
    publish_auxiliary_ = get_parameter("publish_auxiliary").as_bool();
    calibrate_time_ = get_parameter("calibrate_time").as_bool();
    synchronize_time_ = get_parameter("synchronize_time").as_bool();
    error_limit_ = get_parameter("error_limit").as_int();
    error_reset_period_ = get_parameter("error_reset_period").as_double();
    diagnostics_tolerance_ = get_parameter("diagnostics_tolerance").as_double();
    diagnostics_window_time_ = get_parameter("diagnostics_window_time").as_double();
    time_offset_ = get_parameter("time_offset").as_double();

    // 内部変数初期化
    is_connected_ = false;
    is_measurement_started_ = false;
    is_stable_ = false;
    user_latency_ = rclcpp::Duration::from_seconds(time_offset_);

    // メッセージヘッダのframe_id設定
    header_frame_id_ =
      (frame_id_.find_first_not_of('/') == std::string::npos) ? "" : frame_id_.substr(frame_id_.find_first_not_of('/'));
      
    hardware_clock_ = 0.0;
    last_hardware_time_stamp_ = 0;
    hardware_clock_adj_ = 0;
    adj_count_ = 0;

    // PointCloud2 Field設定
    cloud2_.height = 1;
    cloud2_.is_bigendian = false;
    cloud2_.is_dense = false;
    sensor_msgs::PointCloud2Modifier pc2_modifier(cloud2_);
    pc2_modifier.setPointCloud2Fields(4, "x", 1, sensor_msgs::msg::PointField::FLOAT32, "y", 1, 
        sensor_msgs::msg::PointField::FLOAT32, "z", 1, sensor_msgs::msg::PointField::FLOAT32,
        "intensity", 1, sensor_msgs::msg::PointField::FLOAT32);

    // サイクル設定
    if (output_cycle_.compare("frame") == 0)
        cycle_ = CYCLE_FRAME;
    else if (output_cycle_.compare("field") == 0)
        cycle_ = CYCLE_FIELD;
    else if (output_cycle_.compare("line") == 0)
        cycle_ = CYCLE_LINE;
    else
    {
        RCLCPP_ERROR(get_logger(), "Unknown output_cycle value %s", output_cycle_.c_str());
        cycle_ = CYCLE_FRAME;
    }

    // フレーム、フィールド初期値
    prev_frame_ = -1;
    prev_field_ = -1;
}

// Lidarとの接続処理
bool Urg3dNode2::connect()
{
    int result = urg3d_open(&urg_, ip_address_.c_str(), ip_port_);
    if(result < 0){
        RCLCPP_ERROR(get_logger(), "Could not open network Hokuyo 3D LiDAR\n%s:%d\n%s",
        ip_address_.c_str(), ip_port_, &urg_.last_errno);

      return false;
    }

    is_connected_ = true;

    // タイムアウト指定(2000ms)
    urg3d_high_set_blocking_timeout_ms(&urg_, 2000);

    sensor_init();

    // LiDAR情報格納
    vendor_name_ = version_.vendor;
    product_name_ = version_.product;
    device_id_ = version_.serial;
    firmware_version_ = version_.firmware;
    protocol_name_ = version_.protocol;

    std::stringstream ss;
    ss << "Connected to a newtork";
    ss << "scan. Hardware ID: " << device_id_;
    RCLCPP_INFO(get_logger(), "%s", ss.str().c_str());

    // インターレース設定
    result = urg3d_high_blocking_set_horizontal_interlace_count(&urg_, interlace_h_);
    if(result < 0){
        RCLCPP_ERROR(get_logger(), "Could not setting.");
        disconnect();

        return false;
    }
    result = urg3d_high_blocking_set_vertical_interlace_count(&urg_, interlace_v_);
    if(result < 0){
        RCLCPP_ERROR(get_logger(), "Could not setting.");
        disconnect();

        return false;
    }

    return true;
}

// センサ接続時の初期化
bool Urg3dNode2::sensor_init()
{
    int result = 0;
    int retry_count = 0;
    bool is_received = false;

    const int RETRY_MAX_COUNT = 10;

    // データ通信を止める
    while(retry_count < RETRY_MAX_COUNT){
        is_received = ((result = urg3d_high_stop_data(&urg_, URG3D_DISTANCE)) >= 0);
        if(is_received == true){
            result = 0;
            is_received = ((result = urg3d_high_stop_data(&urg_, URG3D_DISTANCE_INTENSITY)) >= 0);
            if(is_received == true){
                result = 0;
                is_received = ((result = urg3d_high_stop_data(&urg_, URG3D_AUXILIARY)) >= 0);
                if(is_received == true){
                    break;
                }
            }
        }
        rclcpp::sleep_for(10ms);
        retry_count++;
    }
    if(is_received == false){
        RCLCPP_ERROR(get_logger(), "Could not init library");
        disconnect();

        return false;
    }

    // buffer clear is needed after stop measurement
    urg3d_low_purge(&urg_);

    result = 0;
    retry_count = 0;
    is_received = false;

    // ブロッキング設定初期化
    while(retry_count < RETRY_MAX_COUNT){
        is_received = ((result = urg3d_high_blocking_init(&urg_)) >= 0);
        if(is_received == true){
            break;
        }
        retry_count++;
    }
    if(is_received == false){
        RCLCPP_ERROR(get_logger(), "Could not init library");
        disconnect();

        return false;
    }

    result = 0;
    retry_count = 0;
    is_received = false;

    // ブロッキング終了処理
    while(retry_count < RETRY_MAX_COUNT){
        is_received = ((result = urg3d_high_blocking_wait_finished_initialize(&urg_)) >= 0);
        if(is_received == true){
            break;
        }
        retry_count++;
    }
    if(is_received == false){
        RCLCPP_ERROR(get_logger(), "Could not init library");
        disconnect();

        return false;
    }

    result = 0;
    retry_count = 0;
    is_received = false;

    // バージョン情報取得
    while(retry_count < RETRY_MAX_COUNT){
        is_received = ((result = urg3d_high_blocking_get_sensor_version(&urg_, &version_)) >= 0);
        if(is_received == true){
            break;
        }
        retry_count++;
    }
    if(is_received == false){
        RCLCPP_ERROR(get_logger(), "Could not init library");
        disconnect();

        return false;
    }

    return true;
}

// Lidarとの切断処理
void Urg3dNode2::disconnect()
{
    if(is_connected_){
        urg3d_close(&urg_);
        is_connected_ = false;
    }
}

// Lidarとの再接続処理
void Urg3dNode2::reconnect()
{
    disconnect();

    connect();
}

// scanスレッド
void Urg3dNode2::scan_thread()
{
    RCLCPP_INFO(get_logger(), "start thread.");

    int result = 0;
    reconnect_count_ = 0;

    while(!close_thread_){

        if(!is_connected_){
            if(!connect()){
                rclcpp::sleep_for(3000ms);
                continue;
            }
        }

        // 調整モード
        if (calibrate_time_) {
            calibrate_system_latency(URG_NODE2_CALIBRATION_MEASUREMENT_TIME);
        }

        // 補助データ配信
        if(publish_auxiliary_ == true){
            int ret = urg3d_high_start_data(&urg_, URG3D_AUXILIARY);
            if(ret < 0){
                RCLCPP_WARN(get_logger(), "Could not start Hokuyo Auxiliary\n");
            }
        }
        else{
            int ret = urg3d_high_stop_data(&urg_, URG3D_AUXILIARY);
            if(ret < 0){
                RCLCPP_WARN(get_logger(), "Could not stop Hokuyo Auxiliary\n");
            }
        }

        // 計測開始
        urg3d_measurement_type_t type;
        if(publish_intensity_ == true){
            type = URG3D_DISTANCE_INTENSITY;
        }
        else{
            type = URG3D_DISTANCE;
        }
        int ret = urg3d_high_start_data(&urg_, type);
        if(ret < 0){
            RCLCPP_WARN(get_logger(), "Could not start Hokuyo measurement\n");
    
            // 再接続処理
            reconnect();
            reconnect_count_++;
    
            continue;
        }
        
        rclcpp::sleep_for(100ms);

        is_measurement_started_ = true;
        error_count_ = 0;
        
        rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
        rclcpp::Time prev_time = system_clock.now();
        rclcpp::Time prev_check_time = system_clock.now();

        bool is_get_data = false;
        int error_check_period = 1;

        while(!close_thread_){
            
            // Inactive状態判定
            rclcpp_lifecycle::State state = get_current_state();
            if (state.label() == "inactive") {
                urg3d_high_stop_data(&urg_, type);
                is_measurement_started_ = false;
                break;
            }
            
            // 計測データ処理
            if(urg3d_next_receive_ready(&urg_)){
                
                is_get_data = true;

                // distance & intensity data
                if(urg3d_high_get_measurement_data(&urg_, &measurement_data_)){                
                    if(prev_frame_ == -1){
                        prev_frame_ = measurement_data_.frame_number;
                    }

                    if(prev_frame_ != -1){
                        // データ格納
                        if(create_scan_message2(cloud2_)){
                            ;
                        }
                        else{
                            RCLCPP_WARN(get_logger(), "Could not get scan.");
                            error_count_++;
                            total_error_count_++;
                        }
                        
                        // 条件を満たした際にpublishする
                        if((cycle_ == CYCLE_FRAME && (prev_frame_ != measurement_data_.frame_number)) ||
                            (cycle_ == CYCLE_FIELD && (measurement_data_.line_number == 0)) ||
                            (cycle_ == CYCLE_LINE)){
  
                            RCLCPP_DEBUG(get_logger(), "publish data.");
                            scan_pub_2->publish(cloud2_);
                            cloud2_.data.clear();
                            prev_frame_ = measurement_data_.frame_number;
                            if(scan_freq_){
                                scan_freq_->tick();
                            }
                        }
                    }
                }
                else if(urg3d_high_get_auxiliary_data(&urg_, &auxiliary_data_) > 0) {
                    if(create_auxiliary_message(imu_, mag_, temp_)){
                        RCLCPP_DEBUG(get_logger(), "publish auxiliary.");
                        imu_pub_->publish(imu_);
                        mag_pub_->publish(mag_);
                        temp_pub_->publish(temp_);
                    }
                    else{
                        RCLCPP_WARN(get_logger(), "Could not get auxiliary.");
                            error_count_++;
                            total_error_count_++;
                    }
                }
                else if(urg3d_low_get_binary(&urg_, &header_, data_, &length_data_) > 0) {
                    // error check
                    if(strncmp(header_.type, "ERR", 3) == 0 || strncmp(header_.type , "_er", 3) == 0){
                        if(header_.status[0] != '0'){
                            break;
                        }
                    }
                }
            }
            else{
                rclcpp::sleep_for(10ms);

                rclcpp::Time current_time = system_clock.now();
                rclcpp::Duration period = current_time - prev_check_time;
                if (period.seconds() >= error_check_period) {
                    if(is_get_data == false){
                        error_count_++;
                        total_error_count_++;
                    }

                    prev_check_time = current_time;
                    is_get_data = false;
                }
            }

            // エラーカウント判定
            if(error_count_ > error_limit_){
                RCLCPP_ERROR(get_logger(), "Error count exceeded limit, reconnecting.");
                // 再接続処理
                reconnect();
                reconnect_count_++;
                break;
            }
            else{
                // エラーカウントのリセット
                rclcpp::Time current_time = system_clock.now();
                rclcpp::Duration period = current_time - prev_time;
                if (period.seconds() >= error_reset_period_) {
                    prev_time = current_time;
                    error_count_ = 0;
                }
            }
        }
    }

    // 切断処理
    disconnect();
}

// スキャントピック作成(PointCloud2型)
bool Urg3dNode2::create_scan_message2(sensor_msgs::msg::PointCloud2 & msg)
{
    if(msg.data.size() == 0){
        // PointCloud2 メッセージ初期化
        msg.header.frame_id = frame_id_;
//        msg.header.stamp = rclcpp::Time(measurement_data_.timestamp_ms);
        
	rclcpp::Clock system_clock(RCL_SYSTEM_TIME);	//add
	rclcpp::Time now = system_clock.now();	//add
	msg.header.stamp = now;			//add
	
        msg.row_step = 0;
        msg.width = 0;
    }

    // ポイント数を調べる
    int point_count = 0;
    for(int spot = 0; spot < measurement_data_.spot_count; ++spot){
        for(int echo = 0; echo < measurement_data_.spots[spot].echo_count; ++echo){
            if(measurement_data_.spots[spot].polar[0].range_m < range_min_){
                continue;
            }
            point_count++;
        }
    }

    // PointCloud2データ格納
    msg.data.resize((msg.width + point_count) * msg.point_step);

    float* data = reinterpret_cast<float*>(&cloud2_.data[0]);
    data += cloud2_.width * cloud2_.point_step / sizeof(float);

    for(int spot = 0; spot < measurement_data_.spot_count; ++spot){
        for(int echo = 0; echo < measurement_data_.spots[spot].echo_count; ++echo){
            if(measurement_data_.spots[spot].polar[0].range_m < range_min_){
                continue;
            }

            *(data++) = measurement_data_.spots[spot].point[0].x_m;
            *(data++) = measurement_data_.spots[spot].point[0].y_m;
            *(data++) = measurement_data_.spots[spot].point[0].z_m;
            *(data++) = measurement_data_.spots[spot].point[0].intensity;

            msg.width++;
        }     
    }
    msg.row_step = msg.width * msg.point_step;

    return true;
}

// スキャントピック作成(auxiliary)
bool Urg3dNode2::create_auxiliary_message(sensor_msgs::msg::Imu & imu, sensor_msgs::msg::MagneticField & mag, sensor_msgs::msg::Temperature & temp)
{
    imu.header.frame_id = frame_id_;
    mag.header.frame_id = frame_id_;
    temp.header.frame_id = frame_id_;
    
    int capturng_record_count = auxiliary_data_.record_count;
    if(capturng_record_count > MAXIMUM_RECORD_TIMES){
        capturng_record_count = MAXIMUM_RECORD_TIMES;
    }
    int latest = auxiliary_data_.record_count - 1;
    urg3d_auxiliary_record_t record = auxiliary_data_.records[latest];

    // Imuデータ
    imu.angular_velocity.x = record.gyro_x * GYRO_FACTOR;
    imu.angular_velocity.y = record.gyro_y * GYRO_FACTOR;
    imu.angular_velocity.z = record.gyro_z * GYRO_FACTOR;
    imu.linear_acceleration.x = record.accel_x * ACCEL_FACTOR;
    imu.linear_acceleration.y = record.accel_y * ACCEL_FACTOR;
    imu.linear_acceleration.z = record.accel_z * ACCEL_FACTOR;

    // MagneticFieldデータ
    mag.magnetic_field.x = record.compass_x * COMPASS_FACTOR;
    mag.magnetic_field.y = record.compass_y * COMPASS_FACTOR;
    mag.magnetic_field.z = record.compass_z * COMPASS_FACTOR;

    // Temperatureデータ
    temp.temperature = (record.temperature / TEMPERATURE_FACTOR) + TEMPERATURE_OFFSET;

    return true;
}

// システムレイテンシの計測
void Urg3dNode2::calibrate_system_latency(size_t num_measurements)
{
    if (!is_connected_) {
        RCLCPP_WARN(get_logger(), "Unable to calibrate time offset. Not Ready.");
        return;
    }

    try {
        RCLCPP_INFO(get_logger(), "Starting calibration. This will take a few seconds.");
        RCLCPP_INFO(get_logger(), "Time calibration is still experimental.");

        system_latency_ = rclcpp::Duration(0ns);

        rclcpp::Duration start_offset = get_native_clock_offset(1);
        rclcpp::Duration prev_offset(0ns);

        std::vector<rclcpp::Duration> time_offsets;
        for (size_t i = 0; i < num_measurements; i++) {
            rclcpp::Duration scan_offset = get_time_stamp_offset(1);
            rclcpp::Duration post_offset = get_native_clock_offset(1);
            rclcpp::Duration adjusted_scan_offset = scan_offset - start_offset;
            rclcpp::Duration adjusted_post_offset = post_offset - start_offset;
            rclcpp::Duration average_offset = rclcpp::Duration::from_seconds(
              (adjusted_post_offset.seconds() + prev_offset.seconds()) / 2.0);
            time_offsets.push_back(adjusted_scan_offset - average_offset);
            prev_offset = adjusted_post_offset;
        }

        // 格納した差をソートし中央値を返す
        std::nth_element(
          time_offsets.begin(),
          time_offsets.begin() + time_offsets.size() / 2, time_offsets.end());
        system_latency_ = time_offsets[time_offsets.size() / 2];
        
        RCLCPP_INFO(
          get_logger(), "Calibration finished. Latency is: %.4f sec.",
          (double)(system_latency_.nanoseconds() * 1e-9));
    }
    catch (const std::runtime_error & e) {
        RCLCPP_WARN(get_logger(), "Could not calibrate time offset: %s", e.what());
        system_latency_ = rclcpp::Duration(0ns);
    }
}

// ROS時刻とLiDAR時刻の差の計算
rclcpp::Duration Urg3dNode2::get_native_clock_offset(size_t num_measurements)
{
    return rclcpp::Duration::from_seconds(0);
}

// システム時刻とLiDAR時刻の差の計算
rclcpp::Duration Urg3dNode2::get_time_stamp_offset(size_t num_measurements)
{
    return rclcpp::Duration::from_seconds(0);
}

// 指数移動平均による動的補正
rclcpp::Time Urg3dNode2::get_synchronized_time(long time_stamp, rclcpp::Time system_time_stamp)
{
    rclcpp::Clock system_clock(RCL_SYSTEM_TIME);
    rclcpp::Time now = system_clock.now();
    return now;
}

// 接続先LiDARが強度出力に対応しているかどうか
bool Urg3dNode2::is_intensity_supported(void)
{
    // 現状は全ての機種が対応
    return true;
}

// 診断情報入力
void Urg3dNode2::populate_diagnostics_status(diagnostic_updater::DiagnosticStatusWrapper & status)
{
    if(!is_connected_){
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Not Connected");
    }

    status.add("IP Address", ip_address_);
    status.add("IP Port", ip_port_);

    if (!is_measurement_started_) {
        diagnostic_msgs::msg::DiagnosticStatus::ERROR, "Not started: ";
    }
    else{
        status.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "Streaming");
    }

    status.add("Vendor Name", vendor_name_);
    status.add("Product Name", product_name_);
    status.add("Firmware Version", firmware_version_);
    status.add("Device ID", device_id_);
    status.add("Protocol name", protocol_name_);
    status.add("Computed Latency", system_latency_.seconds());
    status.add("User Time Offset", user_latency_.seconds());
    //status.add("Device Status", device_status_);
    status.add("Scan Retrieve Error Count", error_count_);
    status.add("Scan Retrieve Total Error Count", total_error_count_);
    status.add("Reconnection Count", reconnect_count_);

    if(publish_auxiliary_){
        status.add("Temperature", temp_.temperature);
    }
}

// スキャンスレッドの開始
void Urg3dNode2::start_thread(void)
{
    close_thread_ = false;
    scan_thread_ = std::thread(std::bind(&Urg3dNode2::scan_thread, this));
}

// スキャンスレッドの停止
void Urg3dNode2::stop_thread(void)
{
    close_thread_ = true;
    if(scan_thread_.joinable()){
        scan_thread_.join();
    }
}

// Diagnosticsの開始
void Urg3dNode2::start_diagnostics(void)
{
    // Diagnostics設定
    diagnostic_updater_.reset(new diagnostic_updater::Updater(this));
    diagnostic_updater_->setHardwareID(device_id_);
    diagnostic_updater_->add("Hardware Status", this, &Urg3dNode2::populate_diagnostics_status);

    // Diagnosticsトピック設定
    if(cycle_ == CYCLE_FRAME){
        diagnostics_freq_ = 1.0 / scan_period_ / interlace_h_ / interlace_v_;
    }
    else if(cycle_ == CYCLE_FIELD){
        diagnostics_freq_ = 1.0 / scan_period_;
    }
    else if(cycle_ == CYCLE_LINE){
        diagnostics_freq_ = 1.0 / scan_period_ * frame_per_lines_;
    }

    scan_freq_.reset(
      new diagnostic_updater::HeaderlessTopicDiagnostic(
        "Point Cloud2",
        *diagnostic_updater_,
        diagnostic_updater::FrequencyStatusParam(
          &diagnostics_freq_, &diagnostics_freq_, diagnostics_tolerance_,
          diagnostics_window_time_)));
}

// Diagnosticsの停止
void Urg3dNode2::stop_diagnostics(void)
{
    // Diagnostics解放
    diagnostic_updater_.reset();
    scan_freq_.reset();

    // 暫定対応
    // Diagnosticsが追加したパラメータの解放
    // diagnostic_updaterを再び生成するとdeclare_parameterが呼ばれエラーになる
    //   https://github.com/ros/diagnostics/pull/227
    if (has_parameter("diagnostic_updater.period")) {
        try {
            undeclare_parameter("diagnostic_updater.period");
        } 
        catch (const std::runtime_error & e) {
            RCLCPP_WARN(get_logger(), "undeclare_parameter failed: %s", e.what());
        }
    }
}

}

#include "rclcpp_components/register_node_macro.hpp"
RCLCPP_COMPONENTS_REGISTER_NODE(urg3d_node2::Urg3dNode2)
