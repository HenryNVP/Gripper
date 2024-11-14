import numpy as np
import matplotlib.pyplot as plt
import skfuzzy as fuzzy
from skfuzzy import control as ctrl

# Class FuzzyController
class FuzzyController:
    def __init__(self):
        # Create the fuzzy system
        self._force_error = ctrl.Antecedent(np.linspace(-0.2, 0.2, 1000), 'force')  # input force [kg]
        self._force_derivative = ctrl.Antecedent(np.linspace(-2, 2, 1000), 'force_derivative')  # derivative of force [kg/s]
        self._actuator = ctrl.Consequent(np.linspace(-0.2, 0.2, 1000), 'actuator')  # variation of actuator [%]

        # Membership Functions for 'force' input
        self._force_error['NL'] = fuzzy.zmf(self._force_error.universe, -0.13, -0.07)
        self._force_error['NM'] = fuzzy.trimf(self._force_error.universe, [-0.1, -0.05, 0])
        self._force_error['ZE'] = fuzzy.trapmf(self._force_error.universe, [-0.05, -0.015, 0.015, 0.05])
        self._force_error['PM'] = fuzzy.trimf(self._force_error.universe, [0, 0.05, 0.1])
        self._force_error['PL'] = fuzzy.smf(self._force_error.universe, 0.07, 0.13)

        # Membership Functions for 'force_derivative' input
        self._force_derivative['NL'] = fuzzy.zmf(self._force_derivative.universe, -1.75, -1.25)
        self._force_derivative['NM'] = fuzzy.trimf(self._force_derivative.universe, [-1.5, -1, -0.5])
        self._force_derivative['ZE'] = fuzzy.trimf(self._force_derivative.universe, [-0.5, 0, 0.5])
        self._force_derivative['PM'] = fuzzy.trimf(self._force_derivative.universe, [0.5, 1, 1.5])
        self._force_derivative['PL'] = fuzzy.smf(self._force_derivative.universe, 1.25, 1.75)

        # Membership Functions for 'actuator' output
        self._actuator['NL'] = fuzzy.zmf(self._actuator.universe, -0.075, -0.025)
        self._actuator['NM'] = fuzzy.trimf(self._actuator.universe, [-0.05, -0.025, 0])
        self._actuator['ZE'] = fuzzy.trapmf(self._actuator.universe, [-0.025, -0.0075, 0.0075, 0.025])
        self._actuator['PM'] = fuzzy.trimf(self._actuator.universe, [0, 0.025, 0.05])
        self._actuator['PL'] = fuzzy.smf(self._actuator.universe,  0.025, 0.075)

        # Create the fuzzy rules (private)
        self._rules = [
            ctrl.Rule(self._force_derivative['NL'], self._actuator['PL']),
            ctrl.Rule(self._force_derivative['PL'], self._actuator['NL']),
            ctrl.Rule(self._force_derivative['NM'] & self._force_error['NL'], self._actuator['NL']),
            ctrl.Rule(self._force_derivative['NM'] & self._force_error['NM'], self._actuator['PM']),
            ctrl.Rule(self._force_derivative['NM'] & self._force_error['ZE'], self._actuator['ZE']),
            ctrl.Rule(self._force_derivative['NM'] & self._force_error['PM'], self._actuator['PM']),
            ctrl.Rule(self._force_derivative['NM'] & self._force_error['PL'], self._actuator['PL']),
            ctrl.Rule(self._force_derivative['ZE'] & self._force_error['NL'], self._actuator['NL']),
            ctrl.Rule(self._force_derivative['ZE'] & self._force_error['NM'], self._actuator['NM']),
            ctrl.Rule(self._force_derivative['ZE'] & self._force_error['ZE'], self._actuator['ZE']),
            ctrl.Rule(self._force_derivative['ZE'] & self._force_error['PM'], self._actuator['PM']),
            ctrl.Rule(self._force_derivative['ZE'] & self._force_error['PL'], self._actuator['PL']),
            ctrl.Rule(self._force_derivative['PM'] & self._force_error['NL'], self._actuator['NL']),
            ctrl.Rule(self._force_derivative['PM'] & self._force_error['NM'], self._actuator['NM']),
            ctrl.Rule(self._force_derivative['PM'] & self._force_error['ZE'], self._actuator['ZE']),
            ctrl.Rule(self._force_derivative['PM'] & self._force_error['PM'], self._actuator['NM']),
            ctrl.Rule(self._force_derivative['PM'] & self._force_error['PL'], self._actuator['PL'])
        ]
        
        # Control system and simulation
        self._actuator_ctrl = ctrl.ControlSystem(self._rules)
        self._actuator_sim = ctrl.ControlSystemSimulation(self._actuator_ctrl)

    def fuzzy_controller(self, force_error_input, force_derivative_input):
        """
        Computes the actuator variation based on the given inputs.
        """
        # Set inputs (force and force_derivative)
        self._actuator_sim.input['force'] = force_error_input
        self._actuator_sim.input['force_derivative'] = force_derivative_input

        # Compute actuator variation
        self._actuator_sim.compute()
        
        # raw_actuator_output = self._actuator_sim.output['actuator']
        # if raw_actuator_output < -0.1:
        #     actuator_output = -0.1
        # elif raw_actuator_output > 0.1:
        #     actuator_output = 0.1
        # else:
        #     actuator_output = raw_actuator_output

        # Return actuator variation output
        # return actuator_output
    
        return self._actuator_sim.output['actuator']

    def plot_membership_functions(self):
        """
        Plots the membership functions for force, force_derivative, and actuator.
        """
        self._force_error.view()
        self._force_derivative.view()
        self._actuator.view()

        plt.show()

    def view_rule_table(self):
        """
        Prints the rule table for the fuzzy system.
        """
        rules = [
            ('force_derivative NL', 'actuator PL'),
            ('force_derivative PL', 'actuator NL'),
            ('force_derivative NM & force NL', 'actuator NL'),
            ('force_derivative NM & force NM', 'actuator PM'),
            ('force_derivative NM & force ZE', 'actuator ZE'),
            ('force_derivative NM & force PM', 'actuator PM'),
            ('force_derivative NM & force PL', 'actuator PL'),
            ('force_derivative ZE & force NL', 'actuator NL'),
            ('force_derivative ZE & force NM', 'actuator NM'),
            ('force_derivative ZE & force ZE', 'actuator ZE'),
            ('force_derivative ZE & force PM', 'actuator PM'),
            ('force_derivative ZE & force PL', 'actuator PL'),
            ('force_derivative PM & force NL', 'actuator NL'),
            ('force_derivative PM & force NM', 'actuator NM'),
            ('force_derivative PM & force ZE', 'actuator ZE'),
            ('force_derivative PM & force PM', 'actuator NM'),
            ('force_derivative PM & force PL', 'actuator PL')
        ]
        
        print("Rule Table:")
        for idx, rule in enumerate(rules, start=1):
            print(f"Rule {idx}: IF {rule[0]} THEN {rule[1]}")


# controller = FuzzyController()
# controller.plot_membership_functions()
# controller.view_rule_table()