within OpalRT.GenUnits.GENROU;
block GENROU_AC8B
  parameter Real partType = 1;
  // GENROU Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real P_gen = 1100 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 342.702 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tdo_s = 0.03 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_p = 0.7 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Tqo_s = 0.04 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real H = 50 "Inertia constant" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd = 0.2 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq = 0.19 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xq_p = 0.06 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S1 = 0.4 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  parameter Real S12 = 0.8 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROU Parameters"));
  // AC8B Parameters
  // This Model is located at system bus IBUS
  parameter Real TR_ex = 0.02 "regulator input filter time constant (sec)" annotation(Dialog(tab = "AC8B Parameters"));
  parameter Real KPR_ex = 10 "(pu) (> 0) voltage regulator proportional gain" annotation(Dialog(tab = "AC8B Parameters"));
  parameter Real KIR_ex = 0.01 "(pu) voltage regulator integral gain" annotation(Dialog(tab = "AC8B Parameters"));
  parameter Real KDR_ex = 2 "(pu) voltage regulator derivative gain" annotation(Dialog(tab = "AC8B Parameters"));
  parameter Real TDR_ex = 2 "voltage regulator derivative channel time constant (sec)" annotation(Dialog(tab = "AC8B Parameters"));
  parameter Real VPIDMAX_ex = 1 "PID maximum limit" annotation(Dialog(tab = "AC8B Parameters"));
  parameter Real VPIDMIN_ex = -1 "PID minimum limit" annotation(Dialog(tab = "AC8B Parameters"));
  parameter Real KA_ex = 10 "(pu) voltage regulator gain" annotation(Dialog(tab = "AC8B Parameters"));
  parameter Real TA_ex = 0.02 "(sec) regulator time constant" annotation(Dialog(tab = "AC8B Parameters"));
  parameter Real VRMAX_ex = 10 "(pu) Voltage regulator output maximum limit" annotation(Dialog(tab = "AC8B Parameters"));
  parameter Real VRMIN_ex = -10 "(pu) Voltage regulator output minimum limit" annotation(Dialog(tab = "AC8B Parameters"));
  parameter Real KC_ex = 0.02 "rectifier loading factor proportional to commutating reactance" annotation(Dialog(tab = "AC8B Parameters"));
  parameter Real KD_ex = 0.02 "demagnetizing factor, function of AC exciter reactances" annotation(Dialog(tab = "AC8B Parameters"));
  parameter Real KE_ex = 1 "exciter constant related fo self-excited field" annotation(Dialog(tab = "AC8B Parameters"));
  parameter Real TE_ex = 0.02 "exciter time constant (>0)" annotation(Dialog(tab = "AC8B Parameters"));
  parameter Real VFEMAX_ex = 10 "exciter field current limit (> 0)" annotation(Dialog(tab = "AC8B Parameters"));
  parameter Real VEMIN_ex = -10 "Minimum exciter voltage output" annotation(Dialog(tab = "AC8B Parameters"));
  parameter Real E1_ex = 4 "Exciter voltages at which exciter saturation is defined" annotation(Dialog(tab = "AC8B Parameters"));
  parameter Real SE_E1_ex = 0.4 "Exciter saturation function value at E1" annotation(Dialog(tab = "AC8B Parameters"));
  parameter Real E2_ex = 5 "Exciter voltages at which exciter saturation is defined" annotation(Dialog(tab = "AC8B Parameters"));
  parameter Real SE_E2_ex = 0.5 "Exciter saturation function value at E2" annotation(Dialog(tab = "AC8B Parameters"));
  //
  OpalRT.Electrical.Control.Excitation.AC8B AC8B1(TR = TR_ex, KPR = KPR_ex, KIR = KIR_ex, KDR = KDR_ex, TDR = TDR_ex, VPIDMAX = VPIDMAX_ex, VPIDMIN = VPIDMIN_ex, KA = KA_ex, TA = TA_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KC = KC_ex, KD = KD_ex, KE = KE_ex, TE = TE_ex, VFEMAX = VFEMAX_ex, VEMIN = VEMIN_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex) annotation(Placement(visible = true, transformation(origin={-20.25,
            11.5}, extent = {{-14.25,
            -14.5},{14.25,14.5}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-53,18}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(ID = M_ID, P_gen = P_gen, Q_gen = Q_gen, Vt_abs = Vt_abs, Vt_ang = Vt_ang, SB = SB, fn = fn, ZSOURCE_RE = ZSOURCE_RE, Tdo_p = Tdo_p, Tdo_s = Tdo_s, Tqo_p = Tqo_p, Tqo_s = Tqo_s, H = H, D = D, Xd = Xd, Xq = Xq, Xd_p = Xd_p, Xq_p = Xq_p, Xd_s = Xd_s, Xl = Xl, S1 = S1, S12 = S12) annotation(Placement(visible = true, transformation(origin = {20, 0}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin={19,23}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-90, 40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin={-71,5}, extent = {{-10, -10}, {10, 10}}, rotation = 0), iconTransformation(origin = {-90, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin={31,-22}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -40}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(AC8B1.dVREF, dVREF) annotation(Line(points = {{-34.5, 2.8}, {-67.0951, 2.8},
          {-67.0951, 3.85604}, {-67.0951, 3.85604}}, color = {0, 0, 127}));
  connect(bus0, genrou1.p) annotation(Line(points = {{31,-22},{20,-22},{20,-15}}));
  connect(genrou1.TRIP, TRIP) annotation(Line(points = {{20,15},{20.2733,15},{
          20.2733,23},{19,23}}, color = {0, 0, 127}));
  connect(AC8B1.EFD0, genrou1.EFD0) annotation (Line(points = {{-6,-0.1},{-1,
          -0.1},{-1,0},{4.7,0}}, color = {0,0,127}));
  connect(AC8B1.EFD, genrou1.EFD) annotation (Line(points = {{-6,2.8},{-1,2.8},
          {-1,4.2},{5,4.2}}, color = {0,0,127}));
  connect(AC8B1.ETERM0, genrou1.ETERM0) annotation (Line(points = {{-6,5.7},{-1,
          5.7},{-1,7.5},{4.7,7.5}}, color = {0,0,127}));
  connect(AC8B1.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{-6,9.47},{
          -1,9.47},{-1,12},{4.7,12}}, color = {0,0,127}));
  connect(genrou1.VI, AC8B1.VI) annotation(Line(points = {{35, 0}, {51.1568, 0},
         {51.1568, 20.0514}, {-5.9126, 20.0514}, {-5.9126, 20.0514}}, color = {0, 0, 127}));
  connect(genrou1.XADIFD, AC8B1.XADIFD) annotation (Line(points = {{35,7.5},{45,
          7.5},{45,31},{-41,31},{-41,18.75},{-34.5,18.75}}, color = {0,0,127}));
  connect(const.y, AC8B1.VUEL) annotation (Line(points = {{-47.5,18},{-43,18},{
          -43,14.4},{-34.5,14.4}}, color = {0,0,127}));
  connect(AC8B1.VOEL, AC8B1.VUEL) annotation (Line(points = {{-34.5,10.34},{-43,
          10.34},{-43,14.4},{-34.5,14.4}}, color = {0,0,127}));
  connect(AC8B1.VOTHSG, AC8B1.VUEL) annotation (Line(points = {{-34.5,6.28},{
          -43,6.28},{-43,14.4},{-34.5,14.4}}, color = {0,0,127}));
  connect(genrou1.PMECH0, genrou1.PMECH) annotation (Line(points = {{5,-12},{1,
          -12},{1,-9},{5,-9}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {1, 1}), graphics={ Rectangle(origin = {-1.36674, 26.4237}, extent = {{-88.3827, 36.4465}, {94.3052, -90.4328}}), Text(origin = {-31.89, -1.02}, extent = {{-48.06, 27.45}, {116.4, -21.07}}, textString = "GENROU_AC8B"), Text(origin = {-28.2761, 26.3138}, extent = {{-48.06, 27.45}, {-11.08, 3.78747}}, textString = "TRIP"), Text(origin = {-28.54, -54.45}, extent = {{-48.06, 27.45}, {-4.6946, 4.93025}}, textString = "dVREF"), Text(origin = {91.3778, -52.8914}, extent = {{-31.4123, 26.3097}, {-4.69, 4.93}}, textString = "Pin")}));
end GENROU_AC8B;
