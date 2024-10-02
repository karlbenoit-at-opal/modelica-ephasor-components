within OpalRT.Electrical.Control.Excitation;
model ESAC1A
  extends OpalRT.Electrical.PartialModel.Exciter;
  parameter Integer IBUS = 100 "Located Bus No.";
  parameter String ID = "M1" "Machine Identifier";
  parameter Real TR = 0.1 "(sec)";
  parameter Real TB = 12 "(sec)";
  parameter Real TC = 10 "(sec)";
  parameter Real KA = 400;
  parameter Real TA = 5 "(sec)";
  parameter Real VAMAX = 5;
  parameter Real VAMIN = -5;
  parameter Real TE = 0.08 "(sec)";
  parameter Real KF = 0.4;
  parameter Real TF = 0.2 "(sec)";
  parameter Real KC = 0.4;
  parameter Real KD = 0.4;
  parameter Real KE = 0.5;
  parameter Real E1 = 4;
  parameter Real SE_E1 = 0.4;
  parameter Real E2 = 5;
  parameter Real SE_E2 = 0.5;
  parameter Real VRMAX = 5;
  parameter Real VRMIN = -5;
  import sat_q = OpalRT.NonElectrical.Math.Nonlinear.computeSaturationQuadratic;
  import OpalRT.Electrical.Control.Excitation.Common.rectifierFunction;
  Modelica.Blocks.Math.Add add1(k2 = -1) annotation(Placement(visible = true, transformation(origin = {-55, -5}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.Math.Continuous.TransferFunction.Lag lag1(T = TR, initType = Modelica.Blocks.Types.Init.InitialOutput, y_start = EC_0) annotation(Placement(visible = true, transformation(origin = {-78, -8}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Math.Product product1 annotation(Placement(visible = true, transformation(origin = {36, 87}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.Math.Continuous.TransferFunctionNonWindup.Integrator_NonWindupLimit non_windup_integrator1(KI = 1 / TE, VRMAX = Modelica.Constants.inf, VRMIN = 0, y_init = VE0) annotation(Placement(visible = true, transformation(origin = {60, 70}, extent = {{10, -10}, {-10, 10}}, rotation = 0)));
  Common.Rectifier filter1 annotation(Placement(visible = true, transformation(origin = {-9, 92}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  NonElectrical.Math.Nonlinear.SaturationQuadratic saturation1(E1 = E1, E2 = E2, SE_E1 = SE_E1, SE_E2 = SE_E2) annotation(Placement(visible = true, transformation(origin = {0, 57}, extent = {{5, -5}, {-5, 5}}, rotation = 0)));
  Modelica.Blocks.Math.Product product2 annotation(Placement(visible = true, transformation(origin = {-40, 61}, extent = {{-5, -5}, {5, 5}}, rotation = 180)));
  Modelica.Blocks.Math.Gain gain3(k = KC) annotation(Placement(visible = true, transformation(origin = {-80, 95}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Math.Add add4(k2 = +1) annotation(Placement(visible = true, transformation(origin = {-71, 50}, extent = {{-5, -5}, {5, 5}}, rotation = 270)));
  Modelica.Blocks.Math.Add add6(k2 = -1, k1 = +1) annotation(Placement(visible = true, transformation(origin = {-10, 30}, extent = {{5, -5}, {-5, 5}}, rotation = 180)));
  Modelica.Blocks.Math.Add add3(k2 = +1) annotation(Placement(visible = true, transformation(origin = {-60, 30}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain1(k = KD) annotation(Placement(visible = true, transformation(origin = {-80, 30}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Math.Gain gain2(k = KE) annotation(Placement(visible = true, transformation(origin = {-74, 70}, extent = {{5, -5}, {-5, 5}}, rotation = 90)));
  Modelica.Blocks.Math.Add3 add31(k3 = +1, k1 = -1) annotation(Placement(visible = true, transformation(origin = {-25, -20}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Continuous.TransferFunction transferfunction1(b = {KF_0, 0}, a = {TF_0, 1}, initType = Modelica.Blocks.Types.Init.InitialOutput) annotation(Placement(visible = true, transformation(origin = {-40, 10}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Nonlinear.Limiter limiter1(uMax = VRMAX, uMin = VRMIN) annotation(Placement(visible = true, transformation(origin = {80, -20}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.Math.Continuous.TransferFunction.LeadLag lead_lag1(TA = TC, TB = TB, initType = Modelica.Blocks.Types.Init.InitialOutput, K = 1, y_start = VA0) annotation(Placement(visible = true, transformation(origin = {-10, -20}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.NonElectrical.Math.Continuous.TransferFunctionNonWindup.Lag_NonWindupLimit lag_non_windup_limit1(KI = KA, TI = TA, VRMAX = VAMAX, VRMIN = VAMIN, y_init = VR0) annotation(Placement(visible = true, transformation(origin = {10, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Math.Max max1 annotation(Placement(visible = true, transformation(origin = {40, -20}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Math.Min min1 annotation(Placement(visible = true, transformation(origin = {60, -20}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Math.Add add2 annotation(Placement(visible = true, transformation(origin = {-45, -55}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Math.Add add5 annotation(Placement(visible = true, transformation(origin = {-30, -80}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = -9999) annotation(Placement(visible = true, transformation(origin = {-70, -70}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant constant1(k = 9999) annotation(Placement(visible = true, transformation(origin = {-60, -90}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.Common.CurrentNormalization division_modified1 annotation(Placement(visible = true, transformation(origin = {-40, 91}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Math.Add add7(k2 = +1) annotation(Placement(visible = true, transformation(origin = {-70, 10}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const1(k = VREF_0) annotation(Placement(visible = true, transformation(origin = {-140, 10}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  Modelica.Blocks.Sources.RealExpression Ecomp(y = ETERM) annotation(Placement(visible = true, transformation(origin = {-135, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
protected
  parameter Real EFD_0(start = 1, fixed = false);
  parameter Real EC_0(start = 1, fixed = false);
  parameter Real VFE0(start = 1, fixed = false);
  parameter Real VREF_0(start = 1, fixed = false);
  parameter Real SE0(start = 1, fixed = false);
  parameter Real FEX0(start = 1, fixed = false);
  parameter Real IN0(start = 1, fixed = false);
  parameter Real IFD0(start = 1, fixed = false);
  parameter Real VE0(start = 1, fixed = false);
  parameter Real VA0(start = 1, fixed = false);
  parameter Real VR0(start = 1, fixed = false);
  parameter Real KF_0(start = 1, fixed = false);
  parameter Real TF_0(start = 1, fixed = false);
initial algorithm
  KF_0 := KF;
  TF_0 := TF;
initial equation
  EFD_0 = EFD0;
  IFD0 = XADIFD;
  EC_0 = ETERM0;
  SE0 = sat_q(VE0, E1, E2, SE_E1, SE_E2);
  VFE0 = IFD0 * KD + (KE + SE0) * VE0;
  IN0 = KC * IFD0 / VE0;
  EFD_0 = VE0 * FEX0;
  FEX0 = rectifierFunction(IN0);
  VR0 = VFE0;
  VA0 = VR0 / KA;
  VREF_0 = EC_0 + VA0;
  VUEL0 = -Modelica.Constants.inf;
  VOEL0 = Modelica.Constants.inf;
equation
  connect(add7.u2, const1.y) annotation(Line(points={{-76,7},{-122.255,7},{-122.255,
          9.79228},{-129,9.79228},{-129,10}},                                                                                                 color = {0, 0, 127}));
  connect(dVREF, add7.u1) annotation(Line(points={{-100,58},{-116.617,58},{-116.617,
          12.7596},{-76,12.7596},{-76,13}},                                                                                                     color = {0, 0, 127}));
  connect(add1.u1, add7.y) annotation(Line(points={{-61,-2},{-62.908,-2},{-62.908,
          10.089},{-64.5,10.089},{-64.5,10}},                                                                                            color = {0, 0, 127}));
  connect(VF, transferfunction1.y) annotation(Line(points = {{100, 80}, {85.3599, 80}, {85.3599, 10}, {85, 10}, {85, 9.79499}, {-34.5, 9.79499}, {-34.5, 10}}, color = {0, 0, 127}));
  connect(division_modified1.IN, filter1.IN) annotation(Line(points = {{-35, 91}, {-29.6128, 91}, {-29.6128, 91.1435}, {-14, 91.1435}, {-14, 90}}, color = {0, 0, 127}));
  connect(division_modified1.VE, non_windup_integrator1.y) annotation(Line(points = {{-44.8, 87.5}, {-47.1526, 87.5}, {-47.1526, 69.2483}, {55, 69.2483}, {55, 70}}, color = {0, 0, 127}));
  connect(division_modified1.IFD, gain3.y) annotation(Line(points = {{-44.9, 94.4}, {-61.5034, 94.4}, {-61.5034, 94.9772}, {-74.5, 94.9772}, {-74.5, 95}}, color = {0, 0, 127}));
  connect(limiter1.y, add6.u1) annotation(Line(points = {{85.5, -20}, {88, -20}, {88, 15.0342}, {-19.59, 15.0342}, {-19.59, 26.6515}, {-16, 26.6515}, {-16, 27}}, color = {0, 0, 127}));
  connect(constant1.y, add5.u2) annotation(Line(points = {{-54.5, -90}, {-43.0524, -90}, {-43.0524, -83.5991}, {-36, -83.5991}, {-36, -83}}, color = {0, 0, 127}));
  connect(const.y, add2.u2) annotation(Line(points = {{-64.5, -70}, {-58.9977, -70}, {-58.9977, -58.7699}, {-51, -58.7699}, {-51, -58}}, color = {0, 0, 127}));
  connect(add5.y, min1.u2) annotation(Line(points = {{-24.5, -80}, {48.7472, -80}, {48.7472, -23.4624}, {54, -23.4624}, {54, -23}}, color = {0, 0, 127}));
  connect(VOEL, add5.u1) annotation(Line(points = {{-100, -8}, {-116.647, -8}, {-116.647, -77.6765}, {-36, -77.6765}, {-36, -77}}, color = {0, 0, 127}));
  connect(add2.y, max1.u2) annotation(Line(points = {{-39.5, -55}, {25.5125, -55}, {25.5125, -23.0068}, {34, -23.0068}, {34, -23}}, color = {0, 0, 127}));
  connect(VUEL, add2.u1) annotation(Line(points = {{-100, 20}, {-109, 20}, {-109, -52.6196}, {-51, -52.6196}, {-51, -52}}, color = {0, 0, 127}));
  connect(min1.y, limiter1.u) annotation(Line(points = {{65.5, -20}, {73.8041, -20}, {73.8041, -20}, {74, -20}}, color = {0, 0, 127}));
  connect(max1.y, min1.u1) annotation(Line(points = {{45.5, -20}, {47.836, -20}, {47.836, -16.8565}, {54, -16.8565}, {54, -17}}, color = {0, 0, 127}));
  connect(lag_non_windup_limit1.y, max1.u1) annotation(Line(points = {{21, -20}, {26.6515, -20}, {26.6515, -16.6287}, {34, -16.6287}, {34, -17}}, color = {0, 0, 127}));
  connect(lag_non_windup_limit1.u, lead_lag1.y) annotation(Line(points = {{-1, -20}, {-4.7836, -20}, {-4.7836, -20}, {-5, -20}}, color = {0, 0, 127}));
  connect(add31.y, lead_lag1.u) annotation(Line(points = {{-19.5, -20}, {-15, -20}}, color = {0, 0, 127}));
  connect(transferfunction1.y, add31.u1) annotation(Line(points = {{-34.5, 10}, {-32.8907, 10}, {-32.8907, -9}, {-33, -9}, {-33, -15.7175}, {-31, -15.7175}, {-31, -16}}, color = {0, 0, 127}));
  connect(transferfunction1.u, add3.y) annotation(Line(points = {{-46, 10}, {-49.8861, 10}, {-49.8861, 29.8405}, {-54.4419, 29.8405}, {-54.4419, 30}, {-54.5, 30}}, color = {0, 0, 127}));
  connect(VOTHSG, add31.u3) annotation(Line(points = {{-100, -36}, {-52.6196, -36}, {-52.6196, -24.6014}, {-31, -24.6014}, {-31, -24}}, color = {0, 0, 127}));
  connect(add1.y, add31.u2) annotation(Line(points = {{-49.5, -5}, {-45.5581, -5}, {-45.5581, -19.8178}, {-31, -19.8178}, {-31, -20}}, color = {0, 0, 127}));
  connect(XADIFD, gain3.u) annotation(Line(points = {{-100, -64}, {-110.707, -64}, {-110.707, 95.101}, {-86, 95.101}, {-86, 95}}, color = {0, 0, 127}));
  connect(add6.y, non_windup_integrator1.u) annotation(Line(points = {{-4.5, 30}, {78.903, 30}, {78.903, 70.1045}, {65, 70.1045}, {65, 70}}, color = {0, 0, 127}));
  connect(add6.u2, add3.y) annotation(Line(points = {{-16, 33}, {-48.5339, 33}, {-48.5339, 29.8015}, {-54.5, 29.8015}, {-54.5, 30}}, color = {0, 0, 127}));
  connect(gain1.u, XADIFD) annotation(Line(points = {{-86, 30}, {-111, 30}, {-111, -64}, {-100, -64}}, color = {0, 0, 127}));
  connect(gain2.y, add4.u2) annotation(Line(points = {{-74, 64.5}, {-74, 60.5695}, {-73.8702, 60.5695}, {-73.8702, 56}, {-74, 56}}, color = {0, 0, 127}));
  connect(non_windup_integrator1.y, gain2.u) annotation(Line(points = {{55, 70}, {-47, 70}, {-47, 79}, {-74, 79}, {-74, 76}}, color = {0, 0, 127}));
  connect(gain1.y, add3.u2) annotation(Line(points = {{-74.5, 30}, {-70.7973, 30}, {-70.7973, 26.8565}, {-66, 26.8565}, {-66, 27}}, color = {0, 0, 127}));
  connect(add4.y, add3.u1) annotation(Line(points = {{-71, 44.5}, {-71, 38.0182}, {-71.0251, 38.0182}, {-71.0251, 32.5513}, {-66, 32.5513}, {-66, 33}}, color = {0, 0, 127}));
  connect(product2.y, add4.u1) annotation(Line(points = {{-45.5, 61}, {-68, 61}, {-68, 56}}, color = {0, 0, 127}));
  connect(non_windup_integrator1.y, product2.u2) annotation(Line(points = {{55, 70}, {-24.5558, 70}, {-24.5558, 63.8246}, {-34, 63.8246}, {-34, 64}}, color = {0, 0, 127}));
  connect(saturation1.y, product2.u1) annotation(Line(points = {{-5, 57}, {-25.0114, 57}, {-25.0114, 57.3576}, {-34, 57.3576}, {-34, 58}}, color = {0, 0, 127}));
  connect(saturation1.u, non_windup_integrator1.y) annotation(Line(points = {{5, 57}, {22.4899, 57}, {22.4899, 69.8106}, {55, 69.8106}, {55, 70}}, color = {0, 0, 127}));
  connect(filter1.FEX, product1.u1) annotation(Line(points = {{-4, 90}, {0.1593, 90}, {0.1593, 90.4656}, {19.045, 90.4656}, {19.045, 90}, {30, 90}}, color = {0, 0, 127}));
  connect(non_windup_integrator1.y, product1.u2) annotation(Line(points = {{55, 70}, {22.8246, 70}, {22.8246, 84}, {30, 84}}, color = {0, 0, 127}));
  connect(product1.y, EFD) annotation(Line(points = {{41.5, 87}, {80, 87}, {80, 95}, {118, 95}, {118, 0}, {100, 0}}, color = {0, 0, 127}));
  connect(lag1.y, add1.u2) annotation(Line(points = {{-73, -8}, {-69.2483, -8}, {-69.2483, -7.74487}, {-61, -7.74487}, {-61, -8}}, color = {0, 0, 127}));
  connect(Ecomp.y, lag1.u) annotation(Line(points = {{-124, -20}, {-86, -20}, {-86, -8}, {-83, -8}}, color = {0, 0, 127}));
  annotation(Documentation(info = "<html>
<img src=\"modelica://OpalRT/resource/Excitation/ESAC1A.png\"
alt=\"EXAC1A.png\"><br>

</html>"), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1})), Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1}), graphics={  Text(origin = {43.015, 73.0689}, lineColor = {255, 0, 0}, extent = {{-2.68, 2.59}, {2.68, -2.59}}, textString = "VE"), Text(origin = {23.1269, -11.0706}, lineColor = {255, 0, 0}, extent = {{-2.68, 2.59}, {2.68, -2.59}}, textString = "VA"), Text(origin = {-64.33, -13.73}, lineColor = {255, 0, 0}, extent = {{-2.68, 2.59}, {2.68, -2.59}}, textString = "VC"), Text(origin = {8.2137, 76.6399}, lineColor = {255, 0, 0}, extent = {{-2.68, 2.59}, {2.68, -2.59}}, textString = "FEX"), Text(origin = {-56.9819, 81.9312}, lineColor = {255, 0, 0}, extent = {{-2.68, 2.59}, {2.68, -2.59}}, textString = "VE"), Text(origin = {-24.9274, 87.3069}, lineColor = {255, 0, 0}, extent = {{-2.68, 2.59}, {2.68, -2.59}}, textString = "IN"), Text(origin = {-36.25, 35.3991}, lineColor = {255, 0, 0}, extent = {{-2.68, 2.59}, {2.68, -2.59}}, textString = "VFE"), Text(origin = {33.5094, 17.821}, lineColor = {255, 0, 0}, extent = {{-3.5094, 4.179}, {3.5094, -4.179}}, textString = "VR"), Text(origin = {34.4, 4.97}, lineColor = {255, 0, 0}, extent = {{-2.68, 2.59}, {2.68, -2.59}}, textString = "VF"), Text(origin = {70.2779, -44.5331}, lineColor = {255, 0, 0}, extent = {{-3.3, 3.53}, {3.3, -3.53}}, textString = "VF")}));
end ESAC1A;
