

class BTWriter:
    def __init__(self, filepath):
        self.filepath = filepath
        self.plan = []
        self.bt = ""

    def setPlan(self, plan):
        self.plan = plan

    def recreateBTWithPlan(self, plan):
        self.setPlan(plan)
        self.bt = """
<?xml version="1.0"?>
<root main_tree_to_execute="BehaviorTree">
    <!-- ////////// -->
    <BehaviorTree ID="BehaviorTree">
        <Sequence>
            <SubTree ID="PoiScheduler"/>
        </Sequence>
    </BehaviorTree>
    <!-- ////////// -->
    <BehaviorTree ID="PoiScheduler">

            <Fallback>
                """
        self.bt = self.bt + f"""
            <Sequence>
                <Inverter>
                    <Condition ID="ROS2Condition" interface="ROS2SERVICE" isMonitored="false" name="IsPoiDone0"/>
                </Inverter>
                <Action ID="ROS2Action" interface="ROS2SERVICE" isMonitored="false" name="SetPoi0"/>
            </Sequence>
        """
        for step in self.plan:
            self.bt = self.bt + f"""
                <Sequence>
                    <Inverter>
                        <Condition ID="ROS2Condition" interface="ROS2SERVICE" isMonitored="false" name="IsPoiDone"""+ str(step) +""""/>
                    </Inverter>
                    <Action ID="ROS2Action" interface="ROS2SERVICE" isMonitored="false" name="SetPoi"""+ str(step) +""""/>
                </Sequence>
            """
        self.bt = self.bt + """           
                        <Action ID="ROS2Action" interface="ROS2SERVICE" isMonitored="false" name="ResetTourAndFlags"/>
            </Fallback>

    </BehaviorTree>
    <TreeNodesModel>
        <SubTree ID="PoiScheduler"/>
        <Action ID="ROS2Action">
            <input_port name="interface" type="std::string"/>
            <input_port name="isMonitored" type="std::string"/>
        </Action>
        <Condition ID="ROS2Condition">
            <input_port name="interface" type="std::string"/>
            <input_port name="isMonitored" type="std::string"/>
        </Condition>
    </TreeNodesModel>
    <!-- ////////// -->
</root>

        """
        # print (self.bt)

    def write(self):
        self.file = open(self.filepath, 'w')
        self.file.write(self.bt)
        self.file.close()

    