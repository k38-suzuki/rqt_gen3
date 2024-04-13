/**
    @author Kenta Suzuki
*/

#include "rqt_gen3_lite/my_widget.h"

#include <ros/ros.h>
#include <sensor_msgs/Joy.h>
#include <kortex_driver/Base_JointSpeeds.h>
#include <kortex_driver/JointSpeed.h>
#include <kortex_driver/Twist.h>
#include <kortex_driver/TwistCommand.h>
#include <std_msgs/Empty.h>

#include <kortex_driver/ActionNotification.h>
#include <kortex_driver/ActionEvent.h>
#include <kortex_driver/Base_ClearFaults.h>
#include <kortex_driver/ExecuteAction.h>
#include <kortex_driver/GripperMode.h>
#include <kortex_driver/OnNotificationActionTopic.h>
#include <kortex_driver/ReadAction.h>
#include <kortex_driver/SendGripperCommand.h>

#include <QBoxLayout>
#include <QDoubleSpinBox>
#include <QFormLayout>
#include <QGridLayout>
#include <QLabel>
#include <QTabWidget>
#include <QTimer>
#include <QToolButton>

#include <thread>

#define HOME_ACTION_IDENTIFIER 2

namespace {

std::atomic<int> last_action_notification_event{0};

void notification_callback(const kortex_driver::ActionNotification& notif)
{
    last_action_notification_event = notif.action_event;
}

bool wait_for_action_end_or_abort()
{
    while(ros::ok()) {
        if(last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_END) {
            ROS_INFO("Received ACTION_END notification");
            return true;
        } else if (last_action_notification_event.load() == kortex_driver::ActionEvent::ACTION_ABORT) {
            ROS_INFO("Received ACTION_ABORT notification");
            return false;
        }
        ros::spinOnce();
    }
    return false;
}

}

namespace rqt_gen3_lite {

class JointWidget : public QWidget
{
public:
    JointWidget(QWidget* parent = nullptr);

    enum { NumJoints = 6 };

private:
    void on_toolButton_pressed(int arg1, int arg2);
    void on_toolButton_released(int arg1);
    void on_toolButton_toggled(bool checked);
    void on_timer_timeout();

    QDoubleSpinBox* velSpins[NumJoints];

    ros::NodeHandle n;
    ros::Publisher joint_pub;

    double joint_vel[6];
};

class TwistWidget : public QWidget
{
public:
    TwistWidget(QWidget* parent = nullptr);

private:
    void joyCallback(const sensor_msgs::Joy& msg);

    void on_toolButton_toggled(bool checked);
    void on_timer_timeout();

    QDoubleSpinBox* twistSpins[2];

    ros::NodeHandle n;
    ros::Publisher twist_pub;
    ros::Subscriber joy_sub;
    sensor_msgs::Joy latestJoyState;

    int current_map;
    double twist_linear[3];
    double twist_angular[3];
    bool prev_button_state[2];
};

class HomeWidget : public QWidget
{
public:
    HomeWidget(QWidget* parent = nullptr);

private:
    bool clear();
    bool home();
    bool grip(double value);

    ros::NodeHandle n;
    ros::Subscriber action_sub;
    ros::ServiceClient service_client_activate_notif;

    std::string robot_name;
    bool is_successed;
};

class ActionWidget : public QWidget
{
public:
    ActionWidget(QWidget* parent = nullptr);

private:
    void clear();
    void stop();
    void estop();

    ros::NodeHandle n;
    ros::Publisher clear_faults_pub;
    ros::Publisher stop_pub;
    ros::Publisher emergency_stop_pub;
};

class MyWidget::Impl
{
public:
    MyWidget* self;

    Impl(MyWidget* self);
};

MyWidget::MyWidget(QWidget* parent)
    : QWidget(parent)
{
    impl = new Impl(this);
}

MyWidget::Impl::Impl(MyWidget* self)
    : self(self)
{
    self->setWindowTitle("Gen3 lite");

    auto tabWidget = new QTabWidget;
    tabWidget->addTab(new JointWidget, "Joint speed");
    tabWidget->addTab(new TwistWidget, "Twist");

    auto layout2 = new QHBoxLayout;
    layout2->addWidget(new HomeWidget);
    layout2->addWidget(new ActionWidget);
    layout2->addStretch();

    auto layout = new QVBoxLayout;
    layout->addLayout(layout2);
    layout->addWidget(tabWidget);
    layout->addStretch();
    self->setLayout(layout);
}

MyWidget::~MyWidget()
{
    delete impl;
}

JointWidget::JointWidget(QWidget* parent)
    : QWidget(parent)
{
    const QStringList list = {
        "joint 1 [deg/s]", "joint 2 [deg/s]", "joint 3 [deg/s]",
        "joint 4 [deg/s]", "joint 5 [deg/s]", "joint 6 [deg/s]"
    };

    auto gridLayout = new QGridLayout;
    for(int i = 0; i < NumJoints; ++i) {
        joint_vel[i] = 0.0;

        velSpins[i] = new QDoubleSpinBox;
        velSpins[i]->setRange(0.0, 57.3);
        velSpins[i]->setValue(1.0);

        auto button1 = new QToolButton;
        button1->setText("+");
        button1->setFixedWidth(60);
        connect(button1, &QToolButton::pressed, [=](){ on_toolButton_pressed(i, 0); });
        connect(button1, &QToolButton::released, [=](){ on_toolButton_released(i); });

        auto button2 = new QToolButton;
        button2->setText("-");
        button2->setFixedWidth(60);
        connect(button2, &QToolButton::pressed, [=](){ on_toolButton_pressed(i, 1); });
        connect(button2, &QToolButton::released, [=](){ on_toolButton_released(i); });

        gridLayout->addWidget(new QLabel(list.at(i)), i, 0);
        gridLayout->addWidget(velSpins[i], i, 1);
        gridLayout->addWidget(button1, i, 2);
        gridLayout->addWidget(button2, i, 3);
    }

    auto button = new QToolButton;
    button->setIcon(QIcon::fromTheme("network-wireless"));
    button->setCheckable(true);
    connect(button, &QToolButton::toggled,
        [&](bool checked){ on_toolButton_toggled(checked); });

    auto timer = new QTimer(this);
    timer->start(1000.0 / 40.0);
    connect(timer, &QTimer::timeout, [&](){ on_timer_timeout(); });

    auto layout2 = new QHBoxLayout;
    layout2->addWidget(button);
    layout2->addStretch();

    auto layout = new QVBoxLayout;
    layout->addLayout(layout2);
    layout->addLayout(gridLayout);
    layout->addStretch();
    setLayout(layout);
}

void JointWidget::on_toolButton_pressed(int arg1, int arg2)
{
    double value = arg2 == 0 ? 1.0 : -1.0;
    joint_vel[arg1] = value * velSpins[arg1]->value();
}

void JointWidget::on_toolButton_released(int arg1)
{
    joint_vel[arg1] = 0.0;
}

void JointWidget::on_toolButton_toggled(bool checked)
{
    if(checked) {
        joint_pub = n.advertise<kortex_driver::Base_JointSpeeds>("in/joint_velocity", 1);
    } else {
        joint_pub.shutdown();
    }
}

void JointWidget::on_timer_timeout()
{
    if(joint_pub) {
        kortex_driver::Base_JointSpeeds joint_msg;
        joint_msg.joint_speeds.resize(6);
        for(int i = 0; i < joint_msg.joint_speeds.size(); ++i) {
            joint_msg.joint_speeds[i].joint_identifier = i;
            joint_msg.joint_speeds[i].value = joint_vel[i];
            joint_msg.joint_speeds[i].duration = 0;
        }
        joint_pub.publish(joint_msg);
    }
}

TwistWidget::TwistWidget(QWidget* parent)
    : QWidget(parent)
    , current_map(0)
{
    joy_sub = n.subscribe("joy", 1, &TwistWidget::joyCallback, this);

    twist_linear[0] = twist_linear[1] = twist_linear[2] = 0.0;
    twist_angular[0] = twist_angular[1] = twist_angular[2] = 0.0;
    prev_button_state[0] = prev_button_state[1] = false;

    const QStringList list = { "linear [m/s]", "angular [rad/s]" };
    const double upper[] = { 0.25, 0.80 };

    auto formLayout = new QFormLayout;
    for(int i = 0; i < 2; ++i) {
        twistSpins[i] = new QDoubleSpinBox;
        twistSpins[i]->setRange(0.0, upper[i]);
        twistSpins[i]->setValue(upper[i]);
        twistSpins[i]->setSingleStep(0.01);
        formLayout->addRow(new QLabel(list.at(i)), twistSpins[i]);
    }

    auto button = new QToolButton;
    button->setIcon(QIcon::fromTheme("network-wireless"));
    button->setCheckable(true);
    connect(button, &QToolButton::toggled,
        [&](bool checked){ on_toolButton_toggled(checked); });

    auto timer = new QTimer(this);
    timer->start(1000.0 / 40.0);
    connect(timer, &QTimer::timeout, [&](){ on_timer_timeout(); });

    auto layout2 = new QHBoxLayout;
    layout2->addWidget(button);
    layout2->addStretch();

    auto layout = new QVBoxLayout;
    layout->addLayout(layout2);
    layout->addLayout(formLayout);
    layout->addStretch();
    setLayout(layout);
}

void TwistWidget::joyCallback(const sensor_msgs::Joy& msg)
{
    latestJoyState = msg;
}

void TwistWidget::on_toolButton_toggled(bool checked)
{
    if(checked) {
        twist_pub = n.advertise<kortex_driver::TwistCommand>("in/cartesian_velocity", 1);
    } else {
        twist_pub.shutdown();
    }
}

void TwistWidget::on_timer_timeout()
{
    sensor_msgs::Joy joy = latestJoyState;
    twist_linear[0] = twist_linear[1] = twist_linear[2] = 0.0;
    twist_angular[0] = twist_angular[1] = twist_angular[2] = 0.0;
    if(joy.axes.size() && joy.buttons.size()) {
        static const int button_id[] = { 6, 7 };
        for(int i = 0; i < 2; ++i) {
            bool current_state = joy.buttons[button_id[i]];
            if(current_state && !prev_button_state[i]) {
                current_map = current_map == 1 ? 0 : 1;
                ROS_INFO("Move to %s-mode.", current_map == 0 ? "translation" : "orientation");
            }
            prev_button_state[i] = current_state;
        }

        const double linear_vel = twistSpins[0]->value();
        const double angular_vel = twistSpins[1]->value();
        twist_linear[0] = linear_vel * joy.axes[0];
        twist_linear[1] = linear_vel * joy.axes[1] * -1.0;
        twist_linear[2] = linear_vel * joy.axes[6];
        twist_angular[0] = angular_vel * joy.axes[0];
        twist_angular[1] = angular_vel * joy.axes[1] * -1.0;
        twist_angular[2] = angular_vel * joy.axes[6];
    }

    if(twist_pub) {
        kortex_driver::TwistCommand twist_msg;
        if(current_map == 0) {
            twist_msg.twist.linear_x = twist_linear[0];
            twist_msg.twist.linear_y = twist_linear[1];
            twist_msg.twist.linear_z = twist_linear[2];
            twist_msg.twist.angular_x = 0.0;
            twist_msg.twist.angular_y = 0.0;
            twist_msg.twist.angular_z = 0.0;
        } else if(current_map == 1) {
            twist_msg.twist.linear_x = 0.0;
            twist_msg.twist.linear_y = 0.0;
            twist_msg.twist.linear_z = 0.0;
            twist_msg.twist.angular_x = twist_angular[0];
            twist_msg.twist.angular_y = twist_angular[1];
            twist_msg.twist.angular_z = twist_angular[2];
        }
        twist_pub.publish(twist_msg);
    }
}

HomeWidget::HomeWidget(QWidget* parent)
    : QWidget(parent)
    , robot_name("")
    , is_successed(true)
{
    bool is_gripper_present = false;

    // Parameter robot_name
    if(!ros::param::get("~robot_name", robot_name)) {
        std::string error_string = "Parameter robot_name was not specified, defaulting to " + robot_name + " as namespace";
        ROS_WARN("%s", error_string.c_str());
    } else {
        std::string error_string = "Using robot_name " + robot_name + " as namespace";
        ROS_INFO("%s", error_string.c_str());
    }

    // Parameter is_gripper_present
    if(!ros::param::get("/" + robot_name + "/is_gripper_present", is_gripper_present)) {
        std::string error_string = "Parameter /" + robot_name + "/is_gripper_present was not specified, defaulting to " + std::to_string(is_gripper_present);
        ROS_WARN("%s", error_string.c_str());
    } else {
        std::string error_string = "Using is_gripper_present " + std::to_string(is_gripper_present);
        ROS_INFO("%s", error_string.c_str());
    }

    // Subscribe to the Action Topic
    action_sub = n.subscribe("/" + robot_name  + "/action_topic", 1000, ::notification_callback);

    // We need to call this service to activate the Action Notification on the kortex_driver node.
    service_client_activate_notif = n.serviceClient<kortex_driver::OnNotificationActionTopic>("/" + robot_name + "/base/activate_publishing_of_action_topic");
    kortex_driver::OnNotificationActionTopic service_activate_notif;
    if(service_client_activate_notif.call(service_activate_notif)) {
        ROS_INFO("Action notification activated!");
    } else {
        std::string error_string = "Action notification publication failed";
        ROS_ERROR("%s", error_string.c_str());
        is_successed = false;
    }

    auto button1 = new QToolButton;
    button1->setText("Clear");
    connect(button1, &QToolButton::clicked, [&](){ clear(); });
    auto button2 = new QToolButton;
    button2->setText("Home");
    connect(button2, &QToolButton::clicked, [&](){ home(); });

    if(is_gripper_present) {

    }

    auto layout2 = new QHBoxLayout;
    // layout2->addWidget(button1);
    layout2->addWidget(button2);
    layout2->addStretch();

    auto layout = new QVBoxLayout;
    layout->addLayout(layout2);
    layout->addStretch();
    setLayout(layout);
}

bool HomeWidget::clear()
{
    ros::ServiceClient service_client_clear_faults = n.serviceClient<kortex_driver::Base_ClearFaults>("/" + robot_name + "/base/clear_faults");
    kortex_driver::Base_ClearFaults service_clear_faults;

    // Clear the faults
    if(!service_client_clear_faults.call(service_clear_faults)) {
        std::string error_string = "Failed to clear the faults";
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }

    // Wait a bit
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return true;
}

bool HomeWidget::home()
{
    ros::ServiceClient service_client_read_action = n.serviceClient<kortex_driver::ReadAction>("/" + robot_name + "/base/read_action");
    kortex_driver::ReadAction service_read_action;
    last_action_notification_event = 0;

    // The Home Action is used to home the robot. It cannot be deleted and is always ID #2:
    service_read_action.request.input.identifier = HOME_ACTION_IDENTIFIER;

    if(!service_client_read_action.call(service_read_action)) {
        std::string error_string = "Failed to call ReadAction";
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }

    // We can now execute the Action that we read 
    ros::ServiceClient service_client_execute_action = n.serviceClient<kortex_driver::ExecuteAction>("/" + robot_name + "/base/execute_action");
    kortex_driver::ExecuteAction service_execute_action;

    service_execute_action.request.input = service_read_action.response.output;
    
    if(service_client_execute_action.call(service_execute_action)) {
        ROS_INFO("The Home position action was sent to the robot.");
    } else {
        std::string error_string = "Failed to call ExecuteAction";
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }

    return wait_for_action_end_or_abort();
}

bool HomeWidget::grip(double value)
{
    // Initialize the ServiceClient
    ros::ServiceClient service_client_send_gripper_command = n.serviceClient<kortex_driver::SendGripperCommand>("/" + robot_name + "/base/send_gripper_command");
    kortex_driver::SendGripperCommand service_send_gripper_command;

    // Initialize the request
    kortex_driver::Finger finger;
    finger.finger_identifier = 0;
    finger.value = value;
    service_send_gripper_command.request.input.gripper.finger.push_back(finger);
    service_send_gripper_command.request.input.mode = kortex_driver::GripperMode::GRIPPER_POSITION;

    if(service_client_send_gripper_command.call(service_send_gripper_command)) {
        ROS_INFO("The gripper command was sent to the robot.");
    } else {
        std::string error_string = "Failed to call SendGripperCommand";
        ROS_ERROR("%s", error_string.c_str());
        return false;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    return true;
}

ActionWidget::ActionWidget(QWidget* parent)
    : QWidget(parent)
{
    clear_faults_pub = n.advertise<std_msgs::Empty>("in/clear_faults", 1);
    stop_pub = n.advertise<std_msgs::Empty>("in/stop", 1);
    emergency_stop_pub = n.advertise<std_msgs::Empty>("in/emergency_stop", 1);

    auto button1 = new QToolButton;
    button1->setText("Clear");
    connect(button1, &QToolButton::clicked, [&](){ clear(); });
    auto button2 = new QToolButton;
    button2->setText("Stop");
    connect(button2, &QToolButton::clicked, [&](){ stop(); });
    auto button3 = new QToolButton;
    button3->setText("E-Stop");
    connect(button3, &QToolButton::clicked, [&](){ estop(); });

    auto layout2 = new QHBoxLayout;
    layout2->addWidget(button1);
    layout2->addWidget(button2);
    layout2->addWidget(button3);
    layout2->addStretch();

    auto layout = new QVBoxLayout;
    layout->addLayout(layout2);
    layout->addStretch();
    setLayout(layout);
}

void ActionWidget::clear()
{
    std_msgs::Empty dummy;
    clear_faults_pub.publish(dummy);
}

void ActionWidget::stop()
{
    std_msgs::Empty dummy;
    stop_pub.publish(dummy);
}

void ActionWidget::estop()
{
    std_msgs::Empty dummy;
    emergency_stop_pub.publish(dummy);
}

}
