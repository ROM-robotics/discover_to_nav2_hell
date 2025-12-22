### navigator.hpp
```cpp
bool on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
    const std::vector<std::string> & plugin_lib_names,
    const FeedbackUtils & feedback_utils,
    nav2_bt_navigator::NavigatorMuxer * plugin_muxer,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
  {
    auto node = parent_node.lock();
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    feedback_utils_ = feedback_utils;
    plugin_muxer_ = plugin_muxer;

    // get the default behavior tree for this navigator
    std::string default_bt_xml_filename = getDefaultBTFilepath(parent_node);

    // Create the Behavior Tree Action Server for this navigator
    bt_action_server_ = std::make_unique<nav2_behavior_tree::BtActionServer<ActionT>>(
      node,
      getName(),
      plugin_lib_names,
      default_bt_xml_filename,
      std::bind(&Navigator::onGoalReceived, this, std::placeholders::_1),
      std::bind(&Navigator::onLoop, this),
      std::bind(&Navigator::onPreempt, this, std::placeholders::_1),
      std::bind(&Navigator::onCompletion, this, std::placeholders::_1, std::placeholders::_2));

    bool ok = true;
    if (!bt_action_server_->on_configure()) {
      ok = false;
    }

    BT::Blackboard::Ptr blackboard = bt_action_server_->getBlackboard();
    blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", feedback_utils.tf);  // NOLINT
    blackboard->set<bool>("initial_pose_received", false);  // NOLINT
    blackboard->set<int>("number_recoveries", 0);  // NOLINT
    blackboard->set<std::shared_ptr<nav2_util::OdomSmoother>>("odom_smoother", odom_smoother);  // NOLINT

    return configure(parent_node, odom_smoother) && ok;
  }
  ```

  ### navigate_to_pose.cpp
  ```cpp
  bool
NavigateToPoseNavigator::configure(
  rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,
  std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
{
  start_time_ = rclcpp::Time(0);
  auto node = parent_node.lock();

  if (!node->has_parameter("goal_blackboard_id")) {
    node->declare_parameter("goal_blackboard_id", std::string("goal"));
  }

  goal_blackboard_id_ = node->get_parameter("goal_blackboard_id").as_string();

  if (!node->has_parameter("path_blackboard_id")) {
    node->declare_parameter("path_blackboard_id", std::string("path"));
  }

  path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();

  // Odometry smoother object for getting current speed
  odom_smoother_ = odom_smoother;

  self_client_ = rclcpp_action::create_client<ActionT>(node, getName());

  goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigateToPoseNavigator::onGoalPoseReceived, this, std::placeholders::_1));
  return true;
}
```

---

## 📋 Navigator::on_configure() အသေးစိတ် ရှင်းလင်းချက်

### အခန်း (၁) - အခြေခံ Setup

```cpp
auto node = parent_node.lock();
logger_ = node->get_logger();
clock_ = node->get_clock();
feedback_utils_ = feedback_utils;
plugin_muxer_ = plugin_muxer;
```

**လုပ်ဆောင်ချက်:**

- **parent_node.lock()** - WeakPtr ကနေ SharedPtr ကို ပြောင်းတယ်။ Parent node (BtNavigator) ကို access လုပ်ဖို့
- **logger_** - ROS2 logging system အတွက်။ RCLCPP_INFO, RCLCPP_ERROR စတာတွေ သုံးဖို့
- **clock_** - Timing operations အတွက်။ Current time ရယူဖို့၊ timestamps တွေ ဖန်တီးဖို့
- **feedback_utils_** - Transform buffer, frame names, transform tolerance တွေ သိမ်းထားတယ်။ Robot position ရယူဖို့ လိုအပ်တယ်
- **plugin_muxer_** - Multiple navigators (NavigateToPose, NavigateThroughPoses) တွေ plugins share လုပ်ဖို့ resource manager

### အခန်း (၂) - Default Behavior Tree File ရယူခြင်း

```cpp
std::string default_bt_xml_filename = getDefaultBTFilepath(parent_node);
```

**getDefaultBTFilepath() က ဘာလုပ်သလဲ?**

NavigateToPoseNavigator အတွက် implementation:

```cpp
std::string NavigateToPoseNavigator::getDefaultBTFilepath(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node)
{
  auto node = parent_node.lock();
  
  // Parameter ရှိပြီးသားလား စစ်တယ်
  if (!node->has_parameter("default_nav_to_pose_bt_xml")) {
    // မရှိရင် default value declare လုပ်တယ်
    std::string pkg_share_dir = 
      ament_index_cpp::get_package_share_directory("nav2_bt_navigator");
    
    node->declare_parameter<std::string>(
      "default_nav_to_pose_bt_xml",
      pkg_share_dir + "/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml");
  }
  
  std::string default_bt_xml_filename;
  node->get_parameter("default_nav_to_pose_bt_xml", default_bt_xml_filename);
  
  return default_bt_xml_filename;
}
```

**Return တန်ဖိုး:** `/opt/ros/humble/share/nav2_bt_navigator/behavior_trees/navigate_to_pose_w_replanning_and_recovery.xml`

**ဒီ BT file က ဘာတွေ လုပ်သလဲ?**
- Path planning with replanning (လမ်းစဉ် ပြန်တွက်ခြင်း)
- Path following (လမ်းစဉ်အတိုင်း လိုက်ခြင်း)
- Recovery behaviors (ပြဿနာကြုံရင် recovery actions - spin, back up, clear costmap)
- Obstacle avoidance logic

### အခန်း (၃) - BT Action Server ဖန်တီးခြင်း

```cpp
bt_action_server_ = std::make_unique<nav2_behavior_tree::BtActionServer<ActionT>>(
    node,
    getName(),
    plugin_lib_names,
    default_bt_xml_filename,
    std::bind(&Navigator::onGoalReceived, this, std::placeholders::_1),
    std::bind(&Navigator::onLoop, this),
    std::bind(&Navigator::onPreempt, this, std::placeholders::_1),
    std::bind(&Navigator::onCompletion, this, std::placeholders::_1, std::placeholders::_2));
```

**BtActionServer ဆိုတာ ဘာလဲ?**

ROS2 Action Server တစ်ခု + Behavior Tree executor ပေါင်းစပ်ထားတဲ့ component ဖြစ်တယ်။

**Constructor Parameters အကျဉ်းချုပ်:**

| Parameter | တန်ဖိုး | ရည်ရွယ်ချက် |
|-----------|--------|-------------|
| `node` | BtNavigator | Parent lifecycle node |
| `getName()` | "navigate_to_pose" | Action server name |
| `plugin_lib_names` | BT node plugin libraries | Dynamic plugin loading |
| `default_bt_xml_filename` | BT XML file path | Behavior tree structure |
| Callback 1 | `onGoalReceived` | Goal လက်ခံတဲ့အခါ |
| Callback 2 | `onLoop` | BT tick တစ်ခါတိုင်း |
| Callback 3 | `onPreempt` | Goal cancel/preempt လုပ်တဲ့အခါ |
| Callback 4 | `onCompletion` | Navigation ပြီးတဲ့အခါ |

**Callbacks အသေးစိတ်:**

#### 🎯 onGoalReceived Callback
```cpp
std::bind(&Navigator::onGoalReceived, this, std::placeholders::_1)
```

**လုပ်ဆောင်ချက်:**
- User က "navigate_to_pose" action goal ပို့လာတဲ့အခါ trigger ဖြစ်တယ်
- Goal validation (valid ဖြစ်မဖြစ် စစ်တယ်)
- Behavior tree file loading (goal မှာ custom BT specify လုပ်ထားရင် load လုပ်တယ်)
- Goal pose ကို blackboard မှာ သိမ်းတယ်
- Return: `bool` - Accept or reject goal

**ဥပမာ Implementation:**
```cpp
bool Navigator::onGoalReceived(ActionT::Goal::ConstSharedPtr goal) {
    // Child class (NavigateToPoseNavigator) ရဲ့ goalReceived() ကို delegate လုပ်တယ်
    return goalReceived(goal);
}

bool NavigateToPoseNavigator::goalReceived(ActionT::Goal::ConstSharedPtr goal) {
    auto bt_xml_filename = goal->behavior_tree;
    
    // Custom BT file specified ဆိုရင် load လုပ်တယ်
    if (!bt_xml_filename.empty()) {
        if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
            RCLCPP_ERROR(logger_, "BT file not found: %s", bt_xml_filename.c_str());
            return false;  // Reject goal
        }
    }
    
    // Goal pose ကို blackboard မှာ သိမ်းတယ်
    initializeGoalPose(goal);
    
    return true;  // Accept goal
}
```

#### 🔄 onLoop Callback
```cpp
std::bind(&Navigator::onLoop, this)
```

**လုပ်ဆောင်ချက်:**
- Behavior tree execution loop တစ်ခါတိုင်း ခေါ်တယ် (typically 10-20 Hz)
- Navigation feedback publish လုပ်တယ် (လက်ရှိ position, remaining distance, estimated time)
- Progress monitoring and logging

**ဥပမာ Implementation:**
```cpp
void NavigateToPoseNavigator::onLoop() {
    // Robot ရဲ့ လက်ရှိ pose ရယူတယ်
    geometry_msgs::msg::PoseStamped current_pose;
    if (!nav2_util::getCurrentPose(current_pose, *feedback_utils_.tf,
                                   feedback_utils_.global_frame,
                                   feedback_utils_.robot_frame,
                                   feedback_utils_.transform_tolerance))
    {
        RCLCPP_ERROR(logger_, "Could not get robot pose");
        return;
    }
    
    // Goal pose ကို blackboard ကနေ ရယူတယ်
    geometry_msgs::msg::PoseStamped goal_pose;
    blackboard->get("goal", goal_pose);
    
    // Remaining distance တွက်တယ်
    auto feedback = std::make_shared<ActionT::Feedback>();
    feedback->current_pose = current_pose;
    feedback->distance_remaining = 
        nav2_util::geometry_utils::euclidean_distance(current_pose.pose, goal_pose.pose);
    feedback->navigation_time = clock_->now() - start_time_;
    
    // Feedback publish လုပ်တယ်
    bt_action_server_->publishFeedback(feedback);
}
```

#### ⏸️ onPreempt Callback
```cpp
std::bind(&Navigator::onPreempt, this, std::placeholders::_1)
```

**လုပ်ဆောင်ချက်:**
- User က navigation cancel လုပ်တဲ့အခါ
- New goal လက်ခံပြီး current goal ကို replace လုပ်တဲ့အခါ
- Cleanup operations လုပ်ဖို့ အခွင့်အရေးပေးတယ်

**ဥပမာ Implementation:**
```cpp
void Navigator::onPreempt(ActionT::Goal::ConstSharedPtr goal) {
    RCLCPP_INFO(logger_, "Received preempt request");
    
    // New goal လက်ခံပြီး ချက်ချင်း စမလား၊
    // လက်ရှိ navigation ကို ရပ်ပြီး cleanup လုပ်မလား ဆုံးဖြတ်တယ်
}
```

#### ✅ onCompletion Callback
```cpp
std::bind(&Navigator::onCompletion, this, std::placeholders::_1, std::placeholders::_2)
```

**လုပ်ဆောင်ချက်:**
- BT execution ပြီးတဲ့အခါ trigger ဖြစ်တယ်
- Success သို့ Failure result ကို handle လုပ်တယ်
- Final result message ကို action client ကို ပို့တယ်

**Parameters:**
- `std::placeholders::_1` - BT execution result (SUCCESS/FAILURE/CANCELED)
- `std::placeholders::_2` - BT result message

**ဥပမာ Implementation:**
```cpp
void NavigateToPoseNavigator::onCompletion(
    typename ActionT::Result::SharedPtr result,
    const nav2_behavior_tree::BtStatus final_bt_status)
{
    if (final_bt_status == nav2_behavior_tree::BtStatus::SUCCEEDED) {
        RCLCPP_INFO(logger_, "Navigation succeeded!");
        result->result = ActionT::Result::NONE;
    } else {
        RCLCPP_WARN(logger_, "Navigation failed!");
        result->error_code = computeErrorCode(final_bt_status);
    }
}
```

### အခန်း (၄) - BT Action Server Configuration

```cpp
bool ok = true;
if (!bt_action_server_->on_configure()) {
    ok = false;
}
```

**bt_action_server_->on_configure() က ဘာတွေ လုပ်သလဲ?**

1. **Action Server ဖန်တီးခြင်း**
   - ROS2 action server `/navigate_to_pose` ကို create လုပ်တယ်
   - Clients တွေက ဒီ action ကို ခေါ်နိုင်တယ်

2. **BT Plugin Libraries Loading**
   - `plugin_lib_names` ထဲက BT node plugins တွေကို load လုပ်တယ်
   - ဥပမာ: ComputePathToPose, FollowPath, Spin, BackUp, ClearCostmap nodes

3. **Default BT File Parsing**
   - XML file ကို parse လုပ်တယ်
   - BT structure (nodes, connections, logic) ကို memory မှာ build လုပ်တယ်

4. **BT Factory Registration**
   - Loaded plugins တွေကို BT factory မှာ register လုပ်တယ်
   - BT executor က runtime မှာ instantiate လုပ်နိုင်အောင်

**Configuration Success/Failure:**
- `ok = false` ဖြစ်ရင် BT action server setup မအောင်မြင်ဘူး
- Plugins load မဖြစ်ခြင်း သို့ XML parsing error ကြောင့် ဖြစ်နိုင်တယ်

### အခန်း (၅) - Blackboard Setup

```cpp
BT::Blackboard::Ptr blackboard = bt_action_server_->getBlackboard();
blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", feedback_utils.tf);
blackboard->set<bool>("initial_pose_received", false);
blackboard->set<int>("number_recoveries", 0);
blackboard->set<std::shared_ptr<nav2_util::OdomSmoother>>("odom_smoother", odom_smoother);
```

**Blackboard ဆိုတာ ဘာလဲ?**

Behavior Tree nodes တွေ အကြား data share လုပ်ဖို့ **shared memory/data store** တစ်ခု။ Key-value store pattern သုံးတယ်။

**ဘာကြောင့် လိုအပ်သလဲ?**

```
Scenario: Path planning and following

ComputePathToPose Node:
├─ Input: goal pose
├─ Process: Path planning algorithm
└─ Output: Computed path

          ↓ (Blackboard မှာ path သိမ်းတယ်)

FollowPath Node:
├─ Input: path from blackboard
├─ Process: Path tracking controller
└─ Output: Velocity commands
```

BT nodes တွေ independent ဖြစ်ကြတယ်၊ တစ်ခုနဲ့တစ်ခု direct communication မရှိဘူး။ Blackboard က intermediary အဖြစ် လုပ်ဆောင်တယ်။

**Blackboard Entries အသေးစိတ်:**

#### 🗺️ tf_buffer
```cpp
blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", feedback_utils.tf);
```

**အမျိုးအစား:** `std::shared_ptr<tf2_ros::Buffer>`

**ရည်ရွယ်ချက်:**
- BT nodes တွေ coordinate transformations လုပ်ဖို့
- Robot pose ကို map frame ထဲမှာ ရယူဖို့
- Sensor data တွေကို global frame ကို convert လုပ်ဖို့

**သုံးစွဲပုံ ဥပမာ:**
```cpp
// BT node ထဲမှာ
auto tf_buffer = blackboard->get<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer");

geometry_msgs::msg::TransformStamped transform;
transform = tf_buffer->lookupTransform("map", "base_link", tf2::TimePointZero);

// Robot ရဲ့ map frame ထဲက position ရပြီ
```

#### 🎯 initial_pose_received
```cpp
blackboard->set<bool>("initial_pose_received", false);
```

**အမျိုးအစား:** `bool`

**ရည်ရွယ်ချက်:**
- Robot က AMCL သို့ localization system ကနေ initial pose ရပြီလား စစ်ဖို့
- Navigation စမတိုင်းခင် robot က မိမိနေရာကို သိထားဖို့ လိုအပ်တယ်

**အသုံးပြုပုံ:**
```cpp
// BT condition node: CheckInitialPoseReceived
auto initial_pose_received = blackboard->get<bool>("initial_pose_received");

if (!initial_pose_received) {
    return BT::NodeStatus::FAILURE;  // Can't navigate yet
}

return BT::NodeStatus::SUCCESS;  // Proceed with navigation
```

**Value Update:**
- AMCL က `/initialpose` topic မှာ pose publish လုပ်တဲ့အခါ
- Localization system က confidence threshold ကျော်တဲ့အခါ
- `blackboard->set("initial_pose_received", true)` ဖြစ်သွားတယ်

#### 🔄 number_recoveries
```cpp
blackboard->set<int>("number_recoveries", 0);
```

**အမျိုးအစား:** `int`

**ရည်ရွယ်ချက်:**
- Recovery behaviors (spin, backup, clear costmap) ဘယ်နှစ်ခါ execute လုပ်ခဲ့ပြီလဲ track လုပ်ဖို့
- Infinite recovery loops ကို ကာကွယ်ဖို့

**အသုံးပြုပုံ:**
```cpp
// Recovery behavior BT node
auto num_recoveries = blackboard->get<int>("number_recoveries");

if (num_recoveries >= MAX_RECOVERIES) {
    RCLCPP_ERROR(logger_, "Max recoveries exceeded, aborting");
    return BT::NodeStatus::FAILURE;
}

// Execute recovery
executeRecoveryBehavior();

// Increment counter
blackboard->set("number_recoveries", num_recoveries + 1);
```

**Reset Timing:**
- Navigation goal အသစ် လက်ခံတဲ့အခါ 0 ကို reset လုပ်တယ်
- Successful navigation ပြီးတဲ့အခါ 0 ကို reset လုပ်တယ်

#### 🚀 odom_smoother
```cpp
blackboard->set<std::shared_ptr<nav2_util::OdomSmoother>>("odom_smoother", odom_smoother);
```

**အမျိုးအစား:** `std::shared_ptr<nav2_util::OdomSmoother>`

**ရည်ရွယ်ချက်:**
- Robot ရဲ့ လက်ရှိ velocity (linear, angular) ကို ရယူဖို့
- Smoothed speed data ပေးတယ် (noise filtered)

**အသုံးပြုပုံ:**
```cpp
// BT node: CheckSpeed
auto odom_smoother = blackboard->get<std::shared_ptr<nav2_util::OdomSmoother>>("odom_smoother");

nav2_util::OdomSmoother::Velocities current_vel = odom_smoother->getVelocity();

double linear_speed = current_vel.linear_x;  // m/s
double angular_speed = current_vel.angular_z; // rad/s

if (linear_speed < 0.01) {
    // Robot is stopped
}
```

**Use Cases:**
- Speed-dependent behavior switching (slow vs fast navigation modes)
- Safety checks (e.g., don't execute certain behaviors while moving fast)
- Progress monitoring (stuck detection - speed is zero for too long)

### အခန်း (၆) - Child Class Configuration

```cpp
return configure(parent_node, odom_smoother) && ok;
```

**Polymorphic Call:**
- `configure()` က **virtual function** ဖြစ်တယ်
- Child class (NavigateToPoseNavigator) ရဲ့ implementation ကို ခေါ်တယ်
- Navigator base class က common setup လုပ်ပြီး child class က specific setup လုပ်တယ်

**Return Logic:**
```cpp
return configure(parent_node, odom_smoother) && ok;
       └───────────────┬──────────────────┘    └┬┘
                       │                        │
              Child config result          BT server config result
```

- **Both must be true** - AND operation (`&&`)
- Child config သို့ BT server config တစ်ခုခု fail ရင် false return လုပ်တယ်
- BtNavigator က FAILURE callback return လုပ်ပြီး configuration မပြီးဘူး

---

## 📋 NavigateToPoseNavigator::configure() အသေးစိတ် ရှင်းလင်းချက်

### အခန်း (၁) - Initialization

```cpp
start_time_ = rclcpp::Time(0);
auto node = parent_node.lock();
```

**start_time_ Reset:**
- Navigation start time ကို 0 ထားတယ်
- Actual navigation goal လက်ခံတဲ့အခါ current time ကို set လုပ်မယ်
- Navigation duration တွက်ချက်ဖို့ သုံးတယ်

**node Lock:**
- WeakPtr → SharedPtr conversion
- Parent node (BtNavigator) ကို access လုပ်ဖို့

### အခန်း (၂) - Goal Blackboard ID Parameter

```cpp
if (!node->has_parameter("goal_blackboard_id")) {
    node->declare_parameter("goal_blackboard_id", std::string("goal"));
}

goal_blackboard_id_ = node->get_parameter("goal_blackboard_id").as_string();
```

**ရည်ရွယ်ချက်:**
- BT blackboard မှာ goal pose ကို သိမ်းမယ့် key name ကို configure လုပ်ဖို့
- Default: `"goal"`

**ဘာကြောင့် configurable လုပ်ထားသလဲ?**
- Multiple navigation goals (primary goal, intermediate waypoints)
- Different BT trees က different key names သုံးချင်တဲ့အခါ
- Flexibility for advanced use cases

**အသုံးပြုပုံ:**
```cpp
// Goal ကို blackboard မှာ သိမ်းတဲ့အခါ (goalReceived callback)
blackboard->set<PoseStamped>(goal_blackboard_id_, goal->pose);

// BT node တွေ goal ကို ဖတ်တဲ့အခါ
auto goal_pose = blackboard->get<PoseStamped>(goal_blackboard_id_);
```

**Configuration ဥပမာ:**
```yaml
bt_navigator:
  ros__parameters:
    goal_blackboard_id: "goal"              # Standard case
    # goal_blackboard_id: "waypoint_goal"   # Custom case
```

### အခန်း (၃) - Path Blackboard ID Parameter

```cpp
if (!node->has_parameter("path_blackboard_id")) {
    node->declare_parameter("path_blackboard_id", std::string("path"));
}

path_blackboard_id_ = node->get_parameter("path_blackboard_id").as_string();
```

**ရည်ရွယ်ချက်:**
- BT blackboard မှာ computed path ကို သိမ်းမယ့် key name
- Default: `"path"`

**Data Flow:**

```
ComputePathToPose BT Node
    ↓ (Plan path)
blackboard->set<Path>(path_blackboard_id_, computed_path)
    ↓
FollowPath BT Node
    ↓ (Read path)
auto path = blackboard->get<Path>(path_blackboard_id_)
    ↓ (Execute path following)
Send velocity commands to robot
```

**Path Data Structure:**
```cpp
nav_msgs::msg::Path {
    std_msgs::Header header;
    std::vector<geometry_msgs::msg::PoseStamped> poses;
}
// Sequence of waypoints from start to goal
```

### အခန်း (၄) - Odometry Smoother Storage

```cpp
odom_smoother_ = odom_smoother;
```

**ရည်ရွယ်ချက်:**
- Member variable မှာ odom smoother ကို သိမ်းထားတယ်
- Navigator methods တွေ (onLoop, goalReceived, etc.) က access လုပ်နိုင်တယ်

**အသုံးပြုပုံ:**
```cpp
// onLoop() callback မှာ
void NavigateToPoseNavigator::onLoop() {
    auto current_velocity = odom_smoother_->getVelocity();
    
    // Feedback မှာ ထည့်ပေးတယ်
    feedback->speed = std::hypot(current_velocity.linear_x, current_velocity.linear_y);
}
```

**Already on Blackboard:**
- Navigator base class က blackboard မှာလည်း သိမ်းထားပြီးသား
- BT nodes တွေက blackboard ကနေ access လုပ်တယ်
- Navigator class methods တွေက member variable ကနေ direct access လုပ်တယ်

### အခန်း (၅) - Self Action Client

```cpp
self_client_ = rclcpp_action::create_client<ActionT>(node, getName());
```

**ActionT Type:** `nav2_msgs::action::NavigateToPose`

**getName() Return:** `"navigate_to_pose"`

**ရည်ရွယ်ချက်:**
- Navigator က မိမိရဲ့ action server ကို goals ပို့နိုင်ဖို့
- Self-referential/recursive navigation

**ဘယ်အချိန်မှာ သုံးသလဲ?**

#### Use Case 1: Goal Pose Topic Subscription
```cpp
void NavigateToPoseNavigator::onGoalPoseReceived(
    const geometry_msgs::msg::PoseStamped::SharedPtr pose)
{
    // RViz 2D Nav Goal ကနေ pose လက်ခံတယ်
    
    // Convert to action goal
    auto goal = nav2_msgs::action::NavigateToPose::Goal();
    goal.pose = *pose;
    goal.behavior_tree = "";  // Use default BT
    
    // Send to own action server
    self_client_->async_send_goal(goal);
}
```

**Flow:**
```
User clicks 2D Nav Goal in RViz
    ↓
Publish to /goal_pose topic
    ↓
onGoalPoseReceived() callback
    ↓
Convert PoseStamped → NavigateToPose::Goal
    ↓
self_client_->async_send_goal()
    ↓
BT Action Server receives goal
    ↓
Execute navigation
```

#### Use Case 2: Multi-Waypoint Navigation (Advanced)
```cpp
void NavigateToPoseNavigator::navigateToWaypoints(
    const std::vector<geometry_msgs::msg::PoseStamped>& waypoints)
{
    for (const auto& waypoint : waypoints) {
        auto goal = nav2_msgs::action::NavigateToPose::Goal();
        goal.pose = waypoint;
        
        // Wait for completion
        auto result_future = self_client_->async_send_goal(goal);
        // ... handle result
    }
}
```

**ဘာကြောင့် self action client လိုအပ်သလဲ?**

- **Topic → Action Conversion:** RViz က PoseStamped topic ပို့တယ်။ Navigator က NavigateToPose action လိုတယ်။
- **Unified Interface:** Navigation logic က action server ထဲမှာပဲ ရှိတယ်။ Topic subscription က action goal အဖြစ် forward လုပ်ရုံပဲ။
- **Consistency:** Direct function calls မလုပ်ဘဲ standard action protocol သုံးတယ်။

### အခန်း (၆) - Goal Pose Subscription

```cpp
goal_sub_ = node->create_subscription<geometry_msgs::msg::PoseStamped>(
    "goal_pose",
    rclcpp::SystemDefaultsQoS(),
    std::bind(&NavigateToPoseNavigator::onGoalPoseReceived, this, std::placeholders::_1));
```

**Subscription Details:**

| Attribute | Value | Description |
|-----------|-------|-------------|
| Topic | `/goal_pose` | Relative (becomes `/bt_navigator/goal_pose`) |
| Type | `geometry_msgs::msg::PoseStamped` | Single pose with timestamp |
| QoS | `SystemDefaultsQoS()` | Default reliability, history settings |
| Callback | `onGoalPoseReceived()` | Member function |

**QoS Settings (SystemDefaultsQoS):**
```cpp
// Typically expands to:
- Reliability: RELIABLE (guaranteed delivery)
- History: KEEP_LAST (keep last N messages)
- Depth: 10 (keep last 10 messages)
- Durability: VOLATILE (don't store for late joiners)
```

**RViz Integration:**

```
RViz 2D Nav Goal Tool
    ↓ (User clicks on map)
geometry_msgs/PoseStamped {
    header: {
        stamp: current_time,
        frame_id: "map"
    },
    pose: {
        position: {x: 5.0, y: 3.0, z: 0.0},
        orientation: {x: 0.0, y: 0.0, z: 0.707, w: 0.707}  // 90° yaw
    }
}
    ↓ (Publish to /goal_pose)
onGoalPoseReceived() callback
    ↓
Convert to NavigateToPose action goal
    ↓
self_client_->async_send_goal()
```

**Alternative: Direct Action Call**

Users can also call the action directly:
```bash
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose "..."
```

Goal pose subscription က RViz users အတွက် convenience feature တစ်ခု။

### အခန်း (၇) - Success Return

```cpp
return true;
```

**Meaning:**
- NavigateToPoseNavigator configuration အောင်မြင်ပြီ
- Navigator::on_configure() က ဒီ result ကို AND operation နဲ့ combine လုပ်တယ်
- BT Action Server config လည်း success ဖြစ်ရမယ်

**Overall Result:**
```cpp
// Navigator::on_configure() မှာ
return configure(parent_node, odom_smoother) && ok;
       └────────────┬─────────────────────┘    └┬┘
                    │                           │
             true (ပြီးပြီ)                BT server OK

Final: true && true = true → Configuration SUCCESS
```

---

## 🎯 Configuration အဆင့်ဆင့် အပြည့်အစုံ Flow

```
BtNavigator::on_configure()
    │
    ├─ Setup: tf, odom_smoother, parameters
    │
    ├─ pose_navigator_->on_configure(...)
    │       │
    │       ├─ Navigator::on_configure() [BASE CLASS]
    │       │       │
    │       │       ├─ Logger, Clock, FeedbackUtils setup
    │       │       ├─ Get default BT file path
    │       │       │   └─ "navigate_to_pose_w_replanning_and_recovery.xml"
    │       │       │
    │       │       ├─ Create BtActionServer
    │       │       │   ├─ Action name: "navigate_to_pose"
    │       │       │   ├─ Load BT plugin libraries
    │       │       │   ├─ Register callbacks:
    │       │       │   │   ├─ onGoalReceived - Validate & accept goal
    │       │       │   │   ├─ onLoop - Publish feedback
    │       │       │   │   ├─ onPreempt - Handle cancellation
    │       │       │   │   └─ onCompletion - Send result
    │       │       │   └─ Parse default BT XML
    │       │       │
    │       │       ├─ bt_action_server_->on_configure()
    │       │       │   ├─ Create action server
    │       │       │   ├─ Load plugins (ComputePath, FollowPath, etc.)
    │       │       │   └─ Build BT structure in memory
    │       │       │
    │       │       ├─ Setup Blackboard
    │       │       │   ├─ "tf_buffer" → Transform buffer
    │       │       │   ├─ "initial_pose_received" → false
    │       │       │   ├─ "number_recoveries" → 0
    │       │       │   └─ "odom_smoother" → Speed info
    │       │       │
    │       │       └─ Call configure(parent_node, odom_smoother)
    │       │               │
    │       │               └─ NavigateToPoseNavigator::configure() [CHILD CLASS]
    │       │                       │
    │       │                       ├─ start_time_ = 0
    │       │                       ├─ goal_blackboard_id_ = "goal"
    │       │                       ├─ path_blackboard_id_ = "path"
    │       │                       ├─ odom_smoother_ = odom_smoother
    │       │                       ├─ self_client_ = create action client
    │       │                       └─ goal_sub_ = subscribe to /goal_pose
    │       │
    │       └─ Return: true/false
    │
    ├─ poses_navigator_->on_configure(...)
    │       └─ (Similar process for NavigateThroughPosesNavigator)
    │
    └─ Return: SUCCESS/FAILURE
```

---

## ✅ Configuration ပြီးပြီးရင် ရရှိတဲ့ Capabilities

### ၁။ Action Server Ready
- `/navigate_to_pose` action server active
- Clients တွေ goals ပို့နိုင်ပြီ
- ROS2 CLI, Python/C++ clients, RViz integration

### ၂။ Behavior Tree Loaded
- Default BT: `navigate_to_pose_w_replanning_and_recovery.xml`
- BT nodes: ComputePath, FollowPath, Recovery behaviors
- Pluggable architecture - custom nodes load လုပ်နိုင်တယ်

### ၃။ Blackboard Initialized
- Shared data store for BT nodes
- Pre-populated: tf_buffer, odom_smoother, counters
- Runtime data: goal, path (added during execution)

### ၄။ Callbacks Registered
- Goal validation and acceptance
- Feedback publishing (progress updates)
- Preemption handling (cancel support)
- Completion result reporting

### ၅။ Topic Integration
- `/goal_pose` subscription active
- RViz 2D Nav Goal tool support
- Topic → Action conversion automatic

### ၆။ Resource Sharing
- Plugin muxer configured
- Multiple navigators can share controllers/planners
- No resource conflicts

---

## 🚀 Ready for Navigation

Configuration ပြီးပြီးရင် Navigator က:
- ✅ Goals လက်ခံနိုင်တယ်
- ✅ Behavior trees execute လုပ်နိုင်တယ်
- ✅ Robot ကို navigate လုပ်နိုင်တယ်
- ✅ Progress feedback ပေးနိုင်တယ်
- ✅ Recovery behaviors handle လုပ်နိုင်တယ်
- ✅ RViz integration လုပ်ဆောင်နိုင်တယ်