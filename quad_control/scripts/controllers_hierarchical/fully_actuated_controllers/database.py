#TODO add an abstract fully-actuated controller to import
import neutral_controller as nc
import simple_bounded_integral_pid_controller as sbipidc
import bounded_integral_pid_controller as bipidc

# data = {
controllers_dictionary = {
"NeutralController": nc.NeutralController,
"SimpleBoundedIntegralPIDController": sbipidc.SimpleBoundedIntegralPIDController,
"BoundedIntegralPIDController": bipidc.BoundedIntegralPIDController
}
