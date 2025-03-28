# This message is used to send information about the motor driver.

# Header information for the message
std_msgs/Header header

# for logging values
float64     cmd_vel_l
float64     cmd_vel_r
float64     vel_l
float64     vel_r
int16       rotpulse_l
int16       rotpulse_r


float64     m_battery_volt_1
int16       m_hall_count_1
int8       m_duty_current_1
float64     m_unit_rpm_1


float64     m_charge_volt_2
int16       m_hall_count_2
int8       m_duty_current_2
float64     m_unit_rpm_2


uint8       m_mode_1
float64     m_lstage_current_1
float64     m_power_current_1


uint8       m_mode_2
float64     m_lstage_current_2
float64     m_power_current_2


float64     m_motor_temp_1
float64     m_substrate_temp_1


float64     m_motor_temp_2
float64     m_substrate_temp_2


string      m_fatal_error_code_1


string      m_fatal_error_code_2

 
string      m_error_code_1


string      m_error_code_2


uint8       m_drive_order_1
float64     m_target_rpm_1
uint8       m_brake_percent_1
uint8       m_target_duty_1


uint8       m_drive_order_2
float64     m_target_rpm_2
uint8       m_brake_percent_2
uint8       m_target_duty_2

