# BT Navigator ၏ လုပ်ဆောင်ပုံနှင့် ဖွဲ့စည်းပုံ (Architecture Overview)

## ၁။ အဓိက Architecture (Overview)
**BT Navigator** ဟာ `nav2_bt_navigator::BtNavigator` ဆိုတဲ့ class ပေါ်မှာ အခြေခံထားပါတယ်။ သူက XML file အနေနဲ့ သိမ်းထားတဲ့ **Behavior Tree** တွေကို ဖတ်ပြီး ရိုဘော့ရဲ့ လုပ်ဆောင်ချက်တွေကို အဆင့်ဆင့် ခိုင်းစေတာဖြစ်ပါတယ်။

## ၂။ အဓိက Function များ (Key Functions)
BT Navigator ရဲ့ ပင်မ အစိတ်အပိုင်းတွေကို အောက်ပါအတိုင်း ခွဲခြားနိုင်ပါတယ် -

*   **`on_configure()`**:
    ဒါက Lifecycle Node ရဲ့ အစိတ်အပိုင်းပါ။ ဒီနေရာမှာ XML File လမ်းကြောင်းတွေကို ဖတ်တာ၊ BT Action Server တွေကို တည်ဆောက်တာနဲ့ Plugins (Action nodes, Condition nodes) တွေကို Register လုပ်တာတွေ လုပ်ဆောင်ပါတယ်။

*   **`on_activate()`**:
    Action Server ကို စတင်ပြီး လက်ခံဖို့ အဆင်သင့်ဖြစ်အောင် လုပ်ပေးပါတယ်။

*   **`MapsToPose()` (သို့မဟုတ် Action Implementation)**:
    အပြင်ကနေ Goal (ပန်းတိုင်) တစ်ခု ပေးလိုက်တဲ့အခါ ဒါမှမဟုတ် Task တစ်ခု ခိုင်းလိုက်တဲ့အခါ ဒီ Function ကနေတစ်ဆင့် BT ကို စတင် Run စေပါတယ်။

*   **`BT::Tree::tickRoot()`**:
    ဒါက Behavior Tree Library ရဲ့ Function ဖြစ်ပါတယ်။ ပေးထားတဲ့ BT ကို တစ်ဆင့်ချင်းစီ (Tick by tick) အလုပ်လုပ်ခိုင်းတဲ့ နေရာပါ။

## ၃။ Threading Model (Thread များ ခွဲခြားထားပုံ)
Nav2 မှာ Thread တွေကို စနစ်တကျ ခွဲထားတာကြောင့် Real-time အခြေအနေတွေမှာ တုံ့ပြန်မှု မြန်ဆန်ပါတယ်။

| Thread အမျိုးအစား | တာဝန်ယူမှု |
| :--- | :--- |
| **Main ROS Thread** | Node ရဲ့ Lifecycle တွေကို ထိန်းချုပ်တာနဲ့ Callback တွေကို လက်ခံတာ လုပ်ပါတယ်။ |
| **Action Server Thread** | Navigation Goal အသစ်တစ်ခု ရောက်လာတဲ့အခါ BT ကို Run ဖို့အတွက် သီးသန့် အလုပ်လုပ်ပေးပါတယ်။ |
| **BT Execution Thread** | BT Navigator ဟာ Tick လုပ်တဲ့အခါမှာ များသောအားဖြင့် Action Server ရဲ့ execute loop ထဲမှာပဲ Run လေ့ရှိပါတယ်။ တစ်ခုထက်ပိုတဲ့ ပန်းတိုင်တွေ (Parallel goals) ရှိရင် Thread ခွဲခြားမှုတွေ ရှိလာနိုင်ပါတယ်။ |

## ၄။ အလုပ်လုပ်ပုံ အဆင့်ဆင့် (Logic Flow)
1.  **Goal Reception**: User ဆီက `MapsToPose` action goal ကို လက်ခံရရှိတယ်။
2.  **Tree Loading**: သတ်မှတ်ထားတဲ့ XML file ကို အခြေခံပြီး BT ကို တည်ဆောက်တယ်။
3.  **Ticking Loop**: BT ကို အဆက်မပြတ် `Tick` လုပ်နေတယ်။
    *   BT ထဲက `ComputePathToPose` (Planner) ကို ခေါ်တယ်။
    *   ပြီးရင် `FollowPath` (Controller) ကို ခေါ်ပြီး ရိုဘော့ကို ရွေ့စေတယ်။
4.  **Feedback**: ရိုဘော့ ဘယ်နားရောက်နေပြီလဲ ဆိုတာကို Feedback အနေနဲ့ ပြန်ပို့ပေးတယ်။
5.  **Completion**: BT က `Success` ပြန်လာရင် Goal ပြီးမြောက်ကြောင်း သတ်မှတ်တယ်။

** ROM က လေ့လာချက် **: BT Navigator က class loader ထဲကို load လုပ်ပေးရမည့်ကောင်ပါ။ သူ့ထဲမှာ တခြား plugins တွေ အများကြီးပါဝင်ပြီး အဲ့ဒီကောင်တွေကို bt navigator က တဆင့် load လုပ်ပေးအုံးမှာဖြစ်ပါတယ်။

---

### 💡 အကြံပြုချက်
Code ကို တကယ်နက်နက်ရှိုင်းရှိုင်း လေ့လာချင်ရင် `nav2_bt_navigator/src/bt_navigator.cpp` ထဲက `BtNavigator::on_configure` နဲ့ `nav2_bt_navigator/src/navigators/` folder ထဲက navigator logic တွေကို အရင်ဖတ်ကြည့်ဖို့ အကြံပေးလိုပါတယ်။
