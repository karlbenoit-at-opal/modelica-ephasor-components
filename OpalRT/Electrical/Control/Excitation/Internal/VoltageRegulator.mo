within OpalRT.Electrical.Control.Excitation.Internal;
block VoltageRegulator
  parameter Real TR "voltage transducer time constant (sec) ";
  parameter Real KVP "voltage regulator proportional gain ";
  parameter Real KVI "voltage regulator integral gain ";
  parameter Real VIMAX "voltage regulator input limit (pu) ";
  parameter Real TA "voltage regulator time constant (sec) ";
  parameter Real TB1 "lag-time constant (sec) ";
  parameter Real TC1 "lead-time constant (sec) ";
  parameter Real TB2 "lag-time constant (sec) ";
  parameter Real TC2 "lead-time constant (sec) ";
  parameter Real VRMAX "maximum controller output (pu) ";
  parameter Real VRMIN "minimum controller output (pu) ";
  parameter Real KF "rate feedback gain ";
  parameter Real TF "rate feedback >0 time constant (sec) ";
  parameter Real TF1 "feedback lead-time constant (sec) ";
  parameter Real TF2 "feedback lag-time constant (sec) ";
  parameter Real FBF "rate feedback signal flag ";
  parameter Real ECOMP0 "initial value of the delay block for ECOMP";
  parameter Real VR0 "initial value of VR";
  Modelica.Blocks.Math.Add3 add31(k2 = +1, k1 = -1) annotation(Placement(visible = true, transformation(origin = {-60, 20}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Math.Add add2(k2 = -1) annotation(Placement(visible = true, transformation(origin = {-40, 0}, extent = {{-5, -5}, {5, 5}}, rotation = 360)));
  Modelica.Blocks.Nonlinear.Limiter limiter1(uMax = VIMAX, uMin = -VIMAX) annotation(Placement(visible = true, transformation(origin = {-20, 0}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.SignalRouting.Multiplexer multiplexer1(n = 3, s = FBF + 1) annotation(Placement(visible = true, transformation(origin = {60, -40}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.Math.Continuous.TransferFunction.Lag lag1(T = TR, y_start = ECOMP0) annotation(Placement(visible = true, transformation(origin = {-60, 60}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput VR annotation(Placement(visible = true, transformation(origin = {100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput EFD annotation(Placement(visible = true, transformation(origin = {100, -60}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {100, -60}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput IFE annotation(Placement(visible = true, transformation(origin = {100, -20}, extent = {{10, -10}, {-10, 10}}, rotation = 0), iconTransformation(origin = {100, 0}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput VREF annotation(Placement(visible = true, transformation(origin = {-100, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput ECOMP annotation(Placement(visible = true, transformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput VS annotation(Placement(visible = true, transformation(origin = {-100, 0}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealInput F annotation(Placement(visible = true, transformation(origin = {-100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-100, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Math.Continuous.TransferFunction.LeadLag lead_lag3(TA = TF1, TB = TF2) annotation(Placement(visible = true, transformation(origin = {-40, -40}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
  Modelica.Blocks.Continuous.TransferFunction transferfunction1(a = {TF, 1}, b = {1, 0}, initType = Modelica.Blocks.Types.Init.InitialOutput, x_start = zeros(1)) annotation(Placement(visible = true, transformation(origin = {-20, -40}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain2(k = KF) annotation(Placement(visible = true, transformation(origin = {20, -40}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.Math.Continuous.TransferFunction.PI_Regulator pi_regulator1(KP = KVP, KI = KVI, y_start = VR0, initType = Modelica.Blocks.Types.Init.InitialOutput) annotation(Placement(visible = true, transformation(origin = {0, 0}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.Math.Continuous.TransferFunctionNonWindup.Lag_NonWindupLimit lag_non_windup_limit1(TI = TA, y_init = VR0, VRMAX = VRMAX, VRMIN = VRMIN) annotation(Placement(visible = true, transformation(origin = {60, 60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Math.Continuous.TransferFunction.TransferFunction2 transfer_function21(b = {TC1 * TC2, TC1 + TC2, 1}, a = {TB1 * TB2, TB1 + TB2, 1}, y_start = VR0) annotation(Placement(visible = true, transformation(origin = {20, 60}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Interfaces.RealOutput VF annotation(Placement(visible = true, transformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {100, 20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(VF, lead_lag3.y) annotation(Line(points = {{100, -80}, {-58.7699, -80}, {-58.7699, -39.8633}, {-45.7859, -39.8633}, {-45.7859, -39.8633}}, color = {0, 0, 127}));
  connect(transfer_function21.y, lag_non_windup_limit1.u) annotation(Line(points = {{25, 60}, {48.7805, 60}, {48.7805, 60}, {49, 60}}, color = {0, 0, 127}));
  connect(pi_regulator1.y, transfer_function21.u) annotation(Line(points = {{5, 0}, {9.24262, 0}, {9.24262, 59.8203}, {15, 59.8203}, {15, 60}}, color = {0, 0, 127}));
  connect(EFD, multiplexer1.u[3]) annotation(Line(points = {{100, -60}, {86.1048, -60}, {86.1048, -40.3189}, {65, -40.3189}, {65, -39.6667}}, color = {0, 0, 127}));
  connect(IFE, multiplexer1.u[2]) annotation(Line(points = {{100, -20}, {81.549, -20}, {81.549, -40.3189}, {65, -40.3189}, {65, -40}}, color = {0, 0, 127}));
  connect(lag_non_windup_limit1.y, multiplexer1.u[1]) annotation(Line(points = {{71, 60}, {71.754, 60}, {71.754, -40.3189}, {65, -40.3189}, {65, -40.3333}}, color = {0, 0, 127}));
  connect(lag_non_windup_limit1.y, VR) annotation(Line(points = {{71, 60}, {90.4328, 60}, {90.4328, 60}, {100, 60}}, color = {0, 0, 127}));
  connect(pi_regulator1.u, limiter1.y) annotation(Line(points = {{-5, 0}, {-14.3437, 0}, {-14.3437, 0}, {-14.5, 0}}, color = {0, 0, 127}));
  connect(gain2.y, transferfunction1.u) annotation(Line(points = {{14.5, -40}, {-13.5318, -40}, {-13.5318, -40}, {-14, -40}}, color = {0, 0, 127}));
  connect(gain2.u, multiplexer1.y) annotation(Line(points = {{26, -40}, {54.9391, -40}, {54.9391, -40}, {55, -40}}, color = {0, 0, 127}));
  connect(transferfunction1.y, lead_lag3.u) annotation(Line(points = {{-25.5, -40}, {-34.912, -40}, {-34.912, -40}, {-35, -40}}, color = {0, 0, 127}));
  connect(lead_lag3.y, add2.u2) annotation(Line(points = {{-45, -40}, {-58.728, -40}, {-58.728, -2.977}, {-46, -2.977}, {-46, -3}}, color = {0, 0, 127}));
  connect(ECOMP, lag1.u) annotation(Line(points = {{-100, 60}, {-65.4685, 60}, {-65.4685, 60}, {-65, 60}}, color = {0, 0, 127}));
  connect(VREF, add31.u3) annotation(Line(points = {{-100, -40}, {-72.7669, -40}, {-72.7669, 15.9041}, {-66, 15.9041}, {-66, 16}}, color = {0, 0, 127}));
  connect(VS, add31.u2) annotation(Line(points = {{-100, 0}, {-78.4314, 0}, {-78.4314, 20.2614}, {-66, 20.2614}, {-66, 20}}, color = {0, 0, 127}));
  connect(lag1.y, add31.u1) annotation(Line(points = {{-55, 60}, {-39.4336, 60}, {-39.4336, 39.8693}, {-73.6383, 39.8693}, {-73.6383, 24.183}, {-66, 24.183}, {-66, 24}}, color = {0, 0, 127}));
  connect(add2.y, limiter1.u) annotation(Line(points = {{-34.5, 0}, {-26.6231, 0}, {-26.6231, 0}, {-26, 0}}, color = {0, 0, 127}));
  connect(add31.y, add2.u1) annotation(Line(points = {{-54.5, 20}, {-53.2026, 20}, {-53.2026, 3.05011}, {-46, 3.05011}, {-46, 3}}, color = {0, 0, 127}));
  annotation(experiment(StartTime = 0, StopTime = 3, Tolerance = 1e-06, Interval = 0.001), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={  Rectangle(origin = {0.12837, -0.12837}, extent = {{-99.7433, 99.7433}, {99.7433, -99.7433}}), Text(origin = {-86.1721, 7.48623}, extent = {{-5.52, -3.98}, {5.52, 3.98}}, textString = "VS"), Text(origin = {-83.606, -75.2006}, extent = {{-11.04, 7.19}, {3.33782, -3.59565}}, textString = "VREF"), Text(origin = {-81.9121, 45.0417}, extent = {{-14.38, -7.19}, {6.16434, 2.56869}}, textString = "F"), Text(origin = {-85.9393, -35.0151}, extent = {{-9.11, 3.98}, {9.11, -3.98}}, textString = "ECOMP"), Text(origin = {94.7407, 42.3648}, extent = {{-13.09, 5.91}, {6.67151, -4.11282}}, textString = "VR"), Text(origin = {98.2046, -73.6872}, extent = {{-13.48, 8.73}, {-1.92436, -3.85195}}, textString = "EFD"), Text(origin = {88.7, -22.46}, extent = {{-8.34, 5.78}, {3.71869, -8.09065}}, textString = "IFE"), Text(origin = {6.93175, 19.5133}, extent = {{-65.47, 28.75}, {52.3763, -24.6422}}, textString = "Voltage"), Text(origin = {-7.98089, -37.5133}, extent = {{-65.47, 28.75}, {86.2659, -13.0889}}, textString = "Regulator"), Text(origin = {76.42, 21.98}, extent = {{-9.91, 5.35}, {9.91, -5.35}}, textString = "VF")}), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics = {Text(origin = {43.96, -75.85}, lineColor = {255, 0, 0}, extent = {{-4.78, 3.19}, {4.78, -3.19}}, textString = "VF")}));
end VoltageRegulator;
