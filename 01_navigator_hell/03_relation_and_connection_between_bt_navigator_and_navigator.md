# BT Navigator နဲ့ Navigator အကြား Relation နဲ့ Connection

## ၁။ BT Navigator Object တွေ နး့ Navigator Function တွေ အကြား Relationship

### 1.1 Class Hierarchy နဲ့ Template Pattern

#### BtNavigator Class (LifecycleNode)
`BtNavigator` က `LifecycleNode` ကို inherit လုပ်ထားပီး main coordinator အဖြစ် အလုပ်လုပ်ပါတယ်။

**Member Objects:**
```cpp
// BtNavigator.hpp မှာ
std::unique_ptr<nav2_bt_navigator::Navigator<nav2_msgs::action::NavigateToPose>> pose_navigator_;
std::unique_ptr<nav2_bt_navigator::Navigator<nav2_msgs::action::NavigateThroughPoses>> poses_navigator_;
nav2_bt_navigator::NavigatorMuxer plugin_muxer_;
```

#### Navigator Template Class
`Navigator<ActionT>` က template class တစ်ခု ဖြစ်ပြီး၊ ကွဲပြားတဲ့ navigation action type တွေအတွက် reusable framework တစ်ခု ဖြစ်ပါတယ်။

**Key Points:**
- `NavigateToPoseNavigator` က `Navigator<nav2_msgs::action::NavigateToPose>` ကို inherit လုပ်ပါတယ်
- `NavigateThroughPosesNavigator` က `Navigator<nav2_msgs::action::NavigateThroughPoses>` ကို inherit လုပ်ပါတယ်
- Template pattern သုံးခြင်းဖြင့် code duplication ကို ရှောင်ရှားပါတယ်

### 1.2 Ownership နဲ့ Lifecycle Management

#### BtNavigator က Navigator တွေကို ဘယ်လို Own လုပ်သလဲ:

**on_configure() မှာ:**
```cpp
// bt_navigator.cpp:131-132
pose_navigator_ = std::make_unique<nav2_bt_navigator::NavigateToPoseNavigator>();
poses_navigator_ = std::make_unique<nav2_bt_navigator::NavigateThroughPosesNavigator>();
```

**ဘာကြောင့် unique_ptr သုံးသလဲ:**
- Exclusive ownership: BtNavigator က navigator တွေကို တစ်ခုတည်းပိုင်ဆိုင်တယ်
- Automatic cleanup: BtNavigator destroy ဖြစ်တဲ့အခါ navigator တွေလည်း automatically cleanup ဖြစ်သွားပါတယ်
- Move semantics: Navigator တွေကို transfer လုပ်နိုင်ပါတယ်

#### Lifecycle Delegation Pattern:

BtNavigator က သူ့ရဲ့ lifecycle callbacks တွေကို navigator တွေဆီက delegate လုပ်ပါတယ်:

```cpp
// bt_navigator.cpp - on_configure() မှာ
if (!pose_navigator_->on_configure(shared_from_this(), plugin_lib_names, 
                                    feedback_utils, &plugin_muxer_, odom_smoother_))
{
    return nav2_util::CallbackReturn::FAILURE;
}

if (!poses_navigator_->on_configure(shared_from_this(), plugin_lib_names, 
                                     feedback_utils, &plugin_muxer_, odom_smoother_))
{
    return nav2_util::CallbackReturn::FAILURE;
}
```

**Pattern:**
1. BtNavigator က configure လုပ်တဲ့အခါ navigator နှစ်ခုစလုံးကို configure လုပ်ပါတယ်
2. တစ်ခုမှာ failure ဖြစ်ရင် တစ်ခုလုံး failure ပြန်ပါတယ် (All-or-nothing)
3. on_activate(), on_deactivate(), on_cleanup() မှာလည်း အတူတူပဲ

### 1.3 Function Call Flow (Detailed)

#### Configuration Phase:
```
BtNavigator::on_configure()
    │
    ├─→ Create shared resources (tf_, odom_smoother_)
    │   
    ├─→ Create FeedbackUtils struct with shared resources
    │
    ├─→ pose_navigator_->on_configure(...)
    │       │
    │       ├─→ Navigator<T>::on_configure()
    │       │       │
    │       │       ├─→ Create BtActionServer<ActionT>
    │       │       │
    │       │       ├─→ Setup blackboard with shared resources
    │       │       │
    │       │       └─→ Call concrete configure()
    │       │
    │       └─→ NavigateToPoseNavigator::configure()
    │               │
    │               ├─→ Setup goal_sub_ (topic subscription)
    │               │
    │               ├─→ Store odom_smoother_
    │               │
    │               └─→ Setup blackboard IDs
    │
    └─→ poses_navigator_->on_configure(...)
            (Same pattern as above)
```

#### Runtime Execution Phase:
```
External Goal Request
    │
    ├─→ BtActionServer receives goal
    │
    ├─→ Navigator::onGoalReceived() [wrapper]
    │       │
    │       ├─→ Check plugin_muxer_->isNavigating()
    │       │
    │       ├─→ NavigateToPoseNavigator::goalReceived()
    │       │       │
    │       │       ├─→ Load BT XML file
    │       │       │
    │       │       └─→ Initialize goal on blackboard
    │       │
    │       └─→ plugin_muxer_->startNavigating()
    │
    ├─→ BT Execution Loop
    │       │
    │       └─→ Navigator::onLoop() [called repeatedly]
    │               │
    │               └─→ NavigateToPoseNavigator::onLoop()
    │                       │
    │                       ├─→ Get current pose from tf_
    │                       │
    │                       ├─→ Get path from blackboard
    │                       │
    │                       ├─→ Calculate feedback
    │                       │
    │                       └─→ Publish feedback
    │
    └─→ Goal Completion
            │
            ├─→ Navigator::onCompletion() [wrapper]
            │       │
            │       ├─→ plugin_muxer_->stopNavigating()
            │       │
            │       └─→ NavigateToPoseNavigator::goalCompleted()
            │
            └─→ Return result to client
```

### 1.4 Virtual Function Pattern (Template Method Pattern)

Navigator base class မှာ virtual functions တွေက concrete navigator class တွေမှာ implement လုပ်ရပါတယ်:

#### Pure Virtual (Must Implement):
```cpp
virtual std::string getName() = 0;
virtual std::string getDefaultBTFilepath(...) = 0;
virtual bool goalReceived(typename ActionT::Goal::ConstSharedPtr goal) = 0;
virtual void onLoop() = 0;
virtual void onPreempt(typename ActionT::Goal::ConstSharedPtr goal) = 0;
virtual void goalCompleted(...) = 0;
```

#### Virtual with Default Implementation (Optional Override):
```cpp
virtual bool configure(...) { return true; }
virtual bool cleanup() { return true; }
virtual bool activate() { return true; }
virtual bool deactivate() { return true; }
```

**NavigateToPoseNavigator မှာ Implementation:**
```cpp
// navigate_to_pose.hpp
std::string getName() override { return std::string("navigate_to_pose"); }
bool configure(...) override;
bool cleanup() override;
bool goalReceived(ActionT::Goal::ConstSharedPtr goal) override;
void onLoop() override;
void onPreempt(ActionT::Goal::ConstSharedPtr goal) override;
void goalCompleted(...) override;
std::string getDefaultBTFilepath(...) override;
```

## ၂။ Node, Context, နဲ့ Shared Resources

### 2.1 ROS Node Sharing Pattern

#### WeakPtr Pattern for Node Sharing:
```cpp
// navigator.hpp:146
bool on_configure(
    rclcpp_lifecycle::LifecycleNode::WeakPtr parent_node,  // ← WeakPtr သုံးတယ်
    const std::vector<std::string> & plugin_lib_names,
    const FeedbackUtils & feedback_utils,
    NavigatorMuxer * plugin_muxer,
    std::shared_ptr<nav2_util::OdomSmoother> odom_smoother)
{
    auto node = parent_node.lock();  // ← အသုံးပြုတဲ့အခါ lock() လုပ်တယ်
    logger_ = node->get_logger();
    clock_ = node->get_clock();
    ...
}
```

**ဘာကြောင့် WeakPtr သုံးသလဲ:**
- **Circular reference prevention:** Navigator က Node ကို reference လုပ်ပြီး၊ Node ကလည်း Navigator ကို own လုပ်တယ်။ SharedPtr သုံးရင် circular reference ဖြစ်ပြီး memory leak ဖြစ်မယ်
- **Lifetime safety:** Parent node destroy ဖြစ်ပီးရင် navigator က အသုံးပြုချင်တဲ့အခါ lock() က nullptr return ပြီး crash မဖြစ်အောင် ကာကွယ်ပေးတယ်
- **Ownership clarity:** Navigator က node ကို own မလုပ်ဘူးဆိုတာ ပီတ်ပီတ်ရှင်းတယ်

#### BtNavigator pass လုပ်ပုံ:
```cpp
// bt_navigator.cpp:145
pose_navigator_->on_configure(
    shared_from_this(),  // ← BtNavigator ရဲ့ shared_ptr ကို pass လုပ်တယ်
    plugin_lib_names, 
    feedback_utils, 
    &plugin_muxer_, 
    odom_smoother_)
```

### 2.2 TF2 Buffer နဲ့ Transform Sharing

#### Creation and Sharing:
```cpp
// bt_navigator.cpp:111-117 - BtNavigator::on_configure()
tf_ = std::make_shared<tf2_ros::Buffer>(get_clock());
auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
    get_node_base_interface(), get_node_timers_interface());
tf_->setCreateTimerInterface(timer_interface);
tf_->setUsingDedicatedThread(true);
tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_, this, false);
```

**FeedbackUtils struct ကနေ share လုပ်ပုံ:**
```cpp
// bt_navigator.cpp:133-137
nav2_bt_navigator::FeedbackUtils feedback_utils;
feedback_utils.tf = tf_;  // ← shared_ptr copy (reference count တက်တယ်)
feedback_utils.global_frame = global_frame_;
feedback_utils.robot_frame = robot_frame_;
feedback_utils.transform_tolerance = transform_tolerance_;
```

**Navigator မှာ အသုံးပြုပုံ:**
```cpp
// navigator.hpp:178
blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", feedback_utils.tf);
```

**NavigateToPoseNavigator မှာ အသုံးပြုပုံ:**
```cpp
// navigate_to_pose.cpp:119-122
geometry_msgs::msg::PoseStamped current_pose;
nav2_util::getCurrentPose(
    current_pose, *feedback_utils_.tf,  // ← shared tf_ buffer ကို သုံးတယ်
    feedback_utils_.global_frame, 
    feedback_utils_.robot_frame,
    feedback_utils_.transform_tolerance);
```

**Sharing Benefits:**
- **Single source of truth:** TF transform data က တစ်ခုတည်းရှိတယ်
- **Memory efficiency:** TF buffer cache ကို duplicate မလုပ်ရဘူး
- **Thread safety:** tf2_ros::Buffer က internally thread-safe ဖြစ်တယ်
- **Dedicated thread:** setUsingDedicatedThread(true) သုံးခြင်းဖြင့် transform lookups တွေက blocking မဖြစ်ဘူး

### 2.3 Blackboard (BT Context Sharing)

#### Blackboard က ဘာလဲ:
Blackboard က BehaviorTree nodes တွေကြား data share လုပ်ဖို့ key-value store တစ်ခု ဖြစ်ပါတယ်။

#### Blackboard Setup:
```cpp
// navigator.hpp:176-180
BT::Blackboard::Ptr blackboard = bt_action_server_->getBlackboard();
blackboard->set<std::shared_ptr<tf2_ros::Buffer>>("tf_buffer", feedback_utils.tf);
blackboard->set<bool>("initial_pose_received", false);
blackboard->set<int>("number_recoveries", 0);
blackboard->set<std::shared_ptr<nav2_util::OdomSmoother>>("odom_smoother", odom_smoother);
```

#### NavigateToPoseNavigator မှာ Blackboard အသုံးပြုပုံ:
```cpp
// navigate_to_pose.cpp:124
auto blackboard = bt_action_server_->getBlackboard();

try {
    // Get current path from blackboard
    nav_msgs::msg::Path current_path;
    blackboard->get<nav_msgs::msg::Path>(path_blackboard_id_, current_path);
    
    // ... process path ...
}
```

**Goal ကို Blackboard မှာ သိမ်းပုံ:**
```cpp
// navigate_to_pose.cpp:220-228 - initializeGoalPose()
void NavigateToPoseNavigator::initializeGoalPose(ActionT::Goal::ConstSharedPtr goal)
{
    auto blackboard = bt_action_server_->getBlackboard();
    
    // Put goal on blackboard for BT nodes to access
    blackboard->set<geometry_msgs::msg::PoseStamped>(
        goal_blackboard_id_,  // "goal"
        goal->pose);
}
```

#### Blackboard Data Flow:
```
BtNavigator
    │
    ├─→ Creates shared resources (tf_, odom_smoother_)
    │
    └─→ Navigator::on_configure()
            │
            ├─→ Creates BtActionServer
            │       │
            │       └─→ Creates Blackboard
            │
            ├─→ Sets shared resources on Blackboard:
            │   • tf_buffer
            │   • odom_smoother
            │   • initial_pose_received
            │   • number_recoveries
            │
            └─→ BT Nodes access Blackboard:
                    │
                    ├─→ ComputePathToPose reads "goal" from Blackboard
                    │
                    ├─→ ComputePathToPose writes "path" to Blackboard
                    │
                    ├─→ FollowPath reads "path" from Blackboard
                    │
                    └─→ Recovery nodes update "number_recoveries"
```

### 2.4 Plugin Muxer (Navigator Coordination)

#### NavigatorMuxer က ဘာလုပ်သလဲ:
တစ်ချိန်မှာ navigator တစ်ခုပဲ active ဖြစ်နိုင်အောင် coordinate လုပ်ပါတယ်။

#### Implementation:
```cpp
// navigator.hpp:49-105
class NavigatorMuxer
{
public:
    NavigatorMuxer() : current_navigator_(std::string("")) {}
    
    bool isNavigating()
    {
        std::scoped_lock l(mutex_);
        return !current_navigator_.empty();
    }
    
    void startNavigating(const std::string & navigator_name)
    {
        std::scoped_lock l(mutex_);
        if (!current_navigator_.empty()) {
            RCLCPP_ERROR(..., "Navigation requested while another navigation task is in progress!");
        }
        current_navigator_ = navigator_name;
    }
    
    void stopNavigating(const std::string & navigator_name)
    {
        std::scoped_lock l(mutex_);
        if (current_navigator_ != navigator_name) {
            RCLCPP_ERROR(..., "Navigation stopped while another navigation task is in progress!");
        } else {
            current_navigator_ = std::string("");
        }
    }

protected:
    std::string current_navigator_;
    std::mutex mutex_;  // ← Thread safety
};
```

#### Usage in BtNavigator:
```cpp
// bt_navigator.hpp:92
nav2_bt_navigator::NavigatorMuxer plugin_muxer_;  // ← Stack object

// bt_navigator.cpp:145
pose_navigator_->on_configure(..., &plugin_muxer_, ...);  // ← Pass pointer
poses_navigator_->on_configure(..., &plugin_muxer_, ...);  // ← Same muxer
```

#### Navigator မှာ အသုံးပြုပုံ:
```cpp
// navigator.hpp:254-269
bool onGoalReceived(typename ActionT::Goal::ConstSharedPtr goal)
{
    if (plugin_muxer_->isNavigating()) {  // ← Check if another navigator is active
        RCLCPP_ERROR(logger_, "Requested navigation while another navigator is processing");
        return false;
    }
    
    bool goal_accepted = goalReceived(goal);
    
    if (goal_accepted) {
        plugin_muxer_->startNavigating(getName());  // ← Register this navigator as active
    }
    
    return goal_accepted;
}

void onCompletion(...)
{
    plugin_muxer_->stopNavigating(getName());  // ← Unregister
    goalCompleted(result, final_bt_status);
}
```

**Muxer Flow:**
```
Time T0: No navigation active
    plugin_muxer_.current_navigator_ = ""

Time T1: NavigateToPose goal received
    │
    ├─→ onGoalReceived() called
    │   ├─→ isNavigating() returns false
    │   ├─→ goalReceived() processes goal
    │   └─→ startNavigating("navigate_to_pose")
    │
    └─→ plugin_muxer_.current_navigator_ = "navigate_to_pose"

Time T2: NavigateThroughPoses goal received (while T1 still active)
    │
    ├─→ onGoalReceived() called
    │   ├─→ isNavigating() returns true  // ← Blocked!
    │   └─→ ERROR: "another navigator is processing"
    │
    └─→ Goal rejected

Time T3: NavigateToPose completes
    │
    ├─→ onCompletion() called
    │   └─→ stopNavigating("navigate_to_pose")
    │
    └─→ plugin_muxer_.current_navigator_ = ""
```

**ဘာကြောင့် pointer (&) pass လုပ်သလဲ:**
- Navigator အားလုံးက single muxer instance ကို share သုံးရမယ်
- Stack allocation ကို သုံးပြီး automatic cleanup
- Ownership က BtNavigator မှာ ရှိတယ်

### 2.5 OdomSmoother Sharing

#### Creation:
```cpp
// bt_navigator.cpp:142
odom_smoother_ = std::make_shared<nav2_util::OdomSmoother>(
    shared_from_this(),  // node
    0.3,                 // filter duration
    odom_topic_);        // topic name
```

#### Multi-level Sharing:
```cpp
// Level 1: Pass to Navigator::on_configure()
pose_navigator_->on_configure(..., odom_smoother_);

// Level 2: Navigator stores it
// navigator.hpp:180
blackboard->set<std::shared_ptr<nav2_util::OdomSmoother>>("odom_smoother", odom_smoother);

// Level 3: NavigateToPoseNavigator stores it
// navigate_to_pose.cpp:47
odom_smoother_ = odom_smoother;

// Level 4: BT nodes access from Blackboard
// (BT nodes ကနေ blackboard မှ "odom_smoother" ကို ဖတ်ပြီး robot speed ရယူတယ်)
```

**Sharing Benefits:**
- **Smoothed velocity:** Raw odometry ကို filter လုပ်ပြီး smooth velocity ရတယ်
- **Feedback calculation:** Navigation feedback မှာ accurate speed information ရတယ်
- **Single subscription:** Odometry topic ကို တစ်ခါပဲ subscribe လုပ်တယ်

### 2.6 BtActionServer Sharing

#### BtActionServer က ဘာလဲ:
Action server interface ကို ပံ့ပိုးပေးပြီး BT execution ကို manage လုပ်ပါတယ်။

#### Creation in Navigator:
```cpp
// navigator.hpp:166-173
bt_action_server_ = std::make_unique<nav2_behavior_tree::BtActionServer<ActionT>>(
    node,
    getName(),                    // Action name: "navigate_to_pose"
    plugin_lib_names,             // BT plugin libraries
    default_bt_xml_filename,      // Default BT XML file
    std::bind(&Navigator::onGoalReceived, this, std::placeholders::_1),
    std::bind(&Navigator::onLoop, this),
    std::bind(&Navigator::onPreempt, this, std::placeholders::_1),
    std::bind(&Navigator::onCompletion, this, std::placeholders::_1, std::placeholders::_2));
```

**Callback Binding:**
- Action server က goal receive လုပ်ရင် → `Navigator::onGoalReceived()` call လုပ်တယ်
- BT loop iteration တိုင်းမှာ → `Navigator::onLoop()` call လုပ်တယ်
- Goal preempt ဖြစ်ရင် → `Navigator::onPreempt()` call လုပ်တယ်
- Goal complete ဖြစ်ရင် → `Navigator::onCompletion()` call လုပ်တယ်

#### Access Pattern:
```cpp
// Navigator provides access to its BtActionServer
// navigator.hpp:237-240
std::unique_ptr<nav2_behavior_tree::BtActionServer<ActionT>> & getActionServer()
{
    return bt_action_server_;
}

// NavigateToPoseNavigator uses it
// navigate_to_pose.cpp:92
if (!bt_action_server_->loadBehaviorTree(bt_xml_filename)) {
    RCLCPP_ERROR(logger_, "BT file not found: %s", bt_xml_filename.c_str());
    return false;
}

// Get blackboard
auto blackboard = bt_action_server_->getBlackboard();
```

### 2.7 Complete Resource Sharing Diagram

```
┌─────────────────────────────────────────────────────────┐
│                    BtNavigator                          │
│                  (LifecycleNode)                        │
│                                                         │
│  Owns:                                                  │
│  • tf_ (shared_ptr<Buffer>)                            │
│  • tf_listener_ (shared_ptr<TransformListener>)        │
│  • odom_smoother_ (shared_ptr<OdomSmoother>)           │
│  • plugin_muxer_ (NavigatorMuxer)                      │
│  • pose_navigator_ (unique_ptr<Navigator<...>>)        │
│  • poses_navigator_ (unique_ptr<Navigator<...>>)       │
└────────────┬────────────────────────┬───────────────────┘
             │                        │
             │ Pass WeakPtr           │ Pass WeakPtr
             │ Pass SharedPtr refs    │ Pass SharedPtr refs
             │ Pass Muxer pointer     │ Pass Muxer pointer
             │                        │
             ▼                        ▼
┌────────────────────────┐  ┌──────────────────────────┐
│  NavigateToPoseNavigator│  │NavigateThroughPoses...   │
│  : Navigator<NavToP>   │  │  : Navigator<NavThP>     │
│                        │  │                          │
│ Owns:                  │  │ Owns:                    │
│ • bt_action_server_    │  │ • bt_action_server_      │
│   (unique_ptr)         │  │   (unique_ptr)           │
│                        │  │                          │
│ References:            │  │ References:              │
│ • feedback_utils_.tf   │  │ • feedback_utils_.tf     │
│   (shared_ptr copy)    │  │   (shared_ptr copy)      │
│ • odom_smoother_       │  │ • odom_smoother_         │
│   (shared_ptr copy)    │  │   (shared_ptr copy)      │
│ • plugin_muxer_        │  │ • plugin_muxer_          │
│   (pointer)            │  │   (pointer)              │
└────────────┬───────────┘  └───────────┬──────────────┘
             │                          │
             │ Both share same          │
             │ • TF Buffer              │
             │ • OdomSmoother           │
             │ • NavigatorMuxer         │
             │                          │
             ▼                          ▼
    ┌────────────────────────────────────────┐
    │         BtActionServer<ActionT>        │
    │                                        │
    │  Owns:                                 │
    │  • action_server_ (rclcpp_action)      │
    │  • bt_blackboard_ (BT::Blackboard)     │
    │                                        │
    │  Blackboard contains:                  │
    │  • "tf_buffer" → shared_ptr<Buffer>    │
    │  • "odom_smoother" → shared_ptr<...>   │
    │  • "goal" → PoseStamped                │
    │  • "path" → Path                       │
    │  • "initial_pose_received" → bool      │
    │  • "number_recoveries" → int           │
    └────────────────┬───────────────────────┘
                     │
                     │ All BT Nodes access
                     │ Blackboard
                     ▼
         ┌───────────────────────────┐
         │     BT Nodes              │
         │                           │
         │ • ComputePathToPose       │
         │ • FollowPath              │
         │ • Recovery Nodes          │
         │ • Condition Nodes         │
         │ • etc.                    │
         └───────────────────────────┘
```

### 2.8 Thread Safety Considerations

#### Mutex Protection:
```cpp
// NavigatorMuxer mutex
std::scoped_lock l(mutex_);  // ← RAII-style locking

// TF Buffer internal thread safety
// tf2_ros::Buffer internally uses locks

// Blackboard access
// BT execution is single-threaded per navigator
```

#### Concurrent Access Patterns:
- **Multiple navigators:** Muxer က mutex ဖြင့် protected
- **TF lookups:** Buffer က internally thread-safe
- **Blackboard:** BT execution က sequential (single-threaded) ဖြစ်တယ်
- **Action servers:** rclcpp_action က internally thread-safe

## ၃။ Communication Patterns Summary

### 3.1 Top-Down (BtNavigator → Navigator)
- Lifecycle management (configure, activate, deactivate, cleanup)
- Resource provisioning (node, tf, odom_smoother, muxer)
- Configuration parameters

### 3.2 Bottom-Up (Navigator → BtNavigator)
- Return status codes (success/failure)
- Indirect through action feedback
- Error logging

### 3.3 Peer-to-Peer (Navigator ↔ Navigator via Muxer)
- Mutual exclusion through NavigatorMuxer
- No direct communication

### 3.4 Shared Context (via Blackboard)
- BT nodes ← Blackboard → Navigator callbacks
- Goal, path, state information
- Shared resources (tf, odom)

## ၄။ Key Design Patterns

### 4.1 Template Method Pattern
- `Navigator<ActionT>` base class defines algorithm structure
- Concrete navigators implement specific steps

### 4.2 Dependency Injection
- BtNavigator creates resources
- Injects them into navigators via constructor/configure

### 4.3 Facade Pattern
- BtNavigator hides navigator complexity
- Provides simple lifecycle interface

### 4.4 Strategy Pattern
- Different navigators (NavigateToPose, NavigateThroughPoses)
- Interchangeable algorithms

### 4.5 Observer Pattern
- Action server callbacks
- BT node updates via blackboard

## ၅။ Memory Management Summary

| Resource | Owner | Type | Lifetime | Share Method |
|----------|-------|------|----------|--------------|
| tf_buffer | BtNavigator | shared_ptr | BtNavigator lifetime | Copy shared_ptr |
| odom_smoother | BtNavigator | shared_ptr | BtNavigator lifetime | Copy shared_ptr |
| plugin_muxer | BtNavigator | Stack object | BtNavigator lifetime | Pass pointer |
| navigators | BtNavigator | unique_ptr | BtNavigator lifetime | Exclusive ownership |
| bt_action_server | Navigator | unique_ptr | Navigator lifetime | Exclusive ownership |
| node | BtNavigator | shared_ptr (this) | ROS manages | WeakPtr to Navigator |
| blackboard | BtActionServer | shared_ptr | BtActionServer lifetime | Ref from Navigator |

---

**ဒီ document က bt_navigator နဲ့ navigator တွေ အကြား ရှိတဲ့ complex relationship တွေ၊ resource sharing patterns တွေ၊ lifecycle management တွေကို အသေးစိတ် ရှင်းပြထားပါတယ်။**
