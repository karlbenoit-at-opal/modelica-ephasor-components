within OpalRT.GenUnits.GENROE;
class GENROE_AC7B
  parameter Real partType = 1;
  // GENROE Parameters
  parameter Integer IBUS = 100 "Located system bus" annotation(Dialog(tab = "General"));
  parameter String M_ID = "M1" "Machine Identifier" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real P_gen = 1100 "Bus Active Power, MW" annotation(Dialog(tab = "General"));
  parameter Real Q_gen = 342.702 "Bus Reactive Power, MVAR" annotation(Dialog(tab = "General"));
  parameter Real Vt_abs = 1.03 "Bus Voltage Magnitude, p.u." annotation(Dialog(tab = "General"));
  parameter Real Vt_ang = -10.96 "Bus Voltage Angle, deg." annotation(Dialog(tab = "General"));
  parameter Real SB = 1000 "Machine Base Power, MVA" annotation(Dialog(tab = "General"));
  parameter Real fn = 50 "Nominal frequency" annotation(Dialog(tab = "General"));
  parameter Real ZSOURCE_RE = 0 "Machine source impedence" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Tdo_p = 7 "d-axis transient time constant" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Tdo_s = 0.03 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Tqo_p = 0.7 "q-axis transient time constant, s" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Tqo_s = 0.04 "d-axis sub-transient time constant, s" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real H = 50 "Inertia constant" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real D = 0 "Speed damping" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Xd = 0.2 "d-axis reactance, p.u." annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Xq = 0.19 "q-axis reactance, p.u." annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Xd_p = 0.06 "d-axis transient reactance, p.u." annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Xq_p = 0.06 "q-axis transient reactance, p.u." annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Xd_s = 0.02 "d-axis sub-transient reactance, p.u." annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real Xl = 0.03 "Reactance due to the leakage flux which does not cross the air gap, p.u." annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real S1 = 0.4 "saturation function value for 1 p.u. input" annotation(Dialog(tab = "GENROE Parameters"));
  parameter Real S12 = 0.8 "saturation function value for 1.2 p.u. input" annotation(Dialog(tab = "GENROE Parameters"));
  // AC7B parameters
  parameter Real TR_ex = 0.04 "(sec) regulator input filter time constant" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KPR_ex = 4.24 "(pu) regulator proportional gain" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KIR_ex = 4.24 "(pu) regulator integral gain" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KDR_ex = 0 "(pu) regulator derivative gain" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real TDR_ex = 0 "(sec) regulator derivative block time constant" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real VRMAX_ex = 5.79 "(pu) regulator output maximum limit" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real VRMIN_ex = -5.79 "(pu) regulator output minimum limit" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KPA_ex = 65.36 "(pu) voltage regulator proportional gain" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KIA_ex = 59.69 "(pu) voltage regulator integral gain" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real VAMAX_ex = 1 "(pu) regulator output maximum limit" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real VAMIN_ex = -0.95 "(pu) regulator output minimum limit" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KP_ex = 4.96 "(pu)" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KL_ex = 10 "(pu)" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KF1_ex = 0.212 "(pu)" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KF2_ex = 0 "(pu)" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KF3_ex = 0 "(pu)" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real TF3_ex = 1 "(sec) time constant (> 0)" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KC_ex = 0.18 "(pu) rectifier loading factor proportional to commutating reactance" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KD_ex = 0.02 "(pu) demagnetizing factor, function of AC exciter reactances" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real KE_ex = 1 "(pu) exciter constant related fo self-excited field" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real TE_ex = 1.1 "(pu) exciter time constant" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real VFEMAX_ex = 6.9 "(pu) exciter field current limit (> 0)" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real VEMIN_ex = 0 "(pu)" annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real E1_ex = 6.67 annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real S_E1_ex = 1.951 annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real E2_ex = 5 annotation(Dialog(tab = "AC7B Parameters"));
  parameter Real S_E2_ex = 0.156 annotation(Dialog(tab = "AC7B Parameters"));
  //-----------------------------------------------------
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin={2,-28}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROE genroe1(
    ID = M_ID,
    P_gen = P_gen,
    Q_gen = Q_gen,
    Vt_abs = Vt_abs,
    Vt_ang = Vt_ang,
    SB = SB,
    fn = fn,
    ZSOURCE_RE = ZSOURCE_RE,
    Tdo_p = Tdo_p,
    Tdo_s = Tdo_s,
    Tqo_p = Tqo_p,
    Tqo_s = Tqo_s,
    H = H,
    D = D,
    Xd = Xd,
    Xq = Xq,
    Xd_p = Xd_p,
    Xq_p = Xq_p,
    Xd_s = Xd_s,
    Xl = Xl,
    S1 = S1,
    S12 = S12) annotation(Placement(visible = true, transformation(origin={-12,8}, extent={{-18,-18},
            {18,18}}, rotation = 0)));
  OpalRT.Electrical.Control.Excitation.AC7B ac7b1(
    TR = TR_ex,
    KPR = KPR_ex,
    KIR = KIR_ex,
    KDR = KDR_ex,
    TDR = TDR_ex,
    VRMAX = VRMAX_ex,
    VRMIN = VRMIN_ex,
    KPA = KPA_ex,
    KIA = KIA_ex,
    VAMAX = VAMAX_ex,
    VAMIN = VAMIN_ex,
    KP = KP_ex,
    KL = KL_ex,
    KF1 = KF1_ex,
    KF2 = KF2_ex,
    KF3 = KF3_ex,
    TF3 = TF3_ex,
    KC = KC_ex,
    KD = KD_ex,
    KE = KE_ex,
    TE = TE_ex,
    VFEMAX = VFEMAX_ex,
    VEMIN = VEMIN_ex,
    E1 = E1_ex,
    S_E1 = S_E1_ex,
    E2 = E2_ex,
    S_E2 = S_E2_ex) annotation(Placement(visible = true, transformation(origin={-56.25,
            20.75}, extent = {{-16.25,
            -17.25},{16.25,17.25}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-94,30}, extent = {{-5, -5}, {5, 5}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin={-106,14}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-200, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin = {-25, 65}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin = {-200, 80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation
  connect(genroe1.VI, ac7b1.VI) annotation(Line(points = {{6, 8}, {18.2977, 8},
         {18.2977, 31.2041}, {-39.8628, 31.2041}, {-39.8628, 31.2041}},
         color = {0, 0, 127}));
  connect(genroe1.p,bus0) annotation(Line(points = {{-12,-10},{-12,-28},{2,-28}}));
  connect(TRIP,genroe1. TRIP) annotation(Line(points = {{-25,65},{-12,65},{-12,
          26}}));
  connect(ac7b1.EFD0, genroe1.EFD0) annotation (Line(points = {{-40,6.95},{-36,
          6.95},{-36,8},{-30.36,8}}, color = {0,0,127}));
  connect(ac7b1.EFD, genroe1.EFD) annotation (Line(points = {{-40,10.4},{-36,
          10.4},{-36,13.04},{-30,13.04}}, color = {0,0,127}));
  connect(ac7b1.ETERM0, genroe1.ETERM0) annotation (Line(points = {{-40,13.85},
          {-36,13.85},{-36,17},{-30.36,17}}, color = {0,0,127}));
  connect(ac7b1.EX_AUX, genroe1.EX_AUX) annotation (Line(points = {{-40,18.335},
          {-36,18.335},{-36,22.4},{-30.36,22.4}}, color = {0,0,127}));
  connect(ac7b1.XADIFD, genroe1.XADIFD) annotation (Line(points = {{-72.5,
          29.375},{-80,29.375},{-80,46},{12,46},{12,17},{6,17}}, color = {0,0,
          127}));
  connect(ac7b1.dVREF, dVREF) annotation(Line(points = {{-72.5, 10.4}, {-105.141, 10.4},
          {-105.141, 11.3111}, {-105.141, 11.3111}}, color = {0, 0, 127}));
  connect(const.y, ac7b1.VUEL) annotation (Line(points = {{-88.5,30},{-82,30},{
          -82,24.2},{-72.5,24.2}}, color = {0,0,127}));
  connect(ac7b1.VOEL, ac7b1.VUEL) annotation (Line(points = {{-72.5,19.37},{-82,
          19.37},{-82,24.2},{-72.5,24.2}}, color = {0,0,127}));
  connect(ac7b1.VOTHSG, ac7b1.VUEL) annotation (Line(points = {{-72.5,14.54},{
          -82,14.54},{-82,24.2},{-72.5,24.2}}, color = {0,0,127}));
  connect(genroe1.PMECH0, genroe1.PMECH) annotation (Line(points = {{-30,-6.4},
          {-36,-6.4},{-36,-2.8},{-30,-2.8}}, color = {0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), experiment(StartTime = 0, StopTime = 20, Tolerance = 1e-06, Interval = 0.01), Icon(coordinateSystem(extent = {{-200, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Rectangle(origin = {-50, 0}, extent = {{-149.774, 100}, {149.774, -100}}), Text(origin = {-77.0165, 1.65977}, extent = {{-72.35, 23.36}, {106.974, -19.9431}}, textString = "GENROE_AC7B"), Text(origin = {77.128, -86.6058}, extent = {{-39.434, 24.942}, {7.19, -8.82}}, textString = "PIN"), Text(origin = {-147.75, 73.49}, extent = {{-39.43, 24.94}, {7.19, -8.82}}, textString = "TRIP"), Text(origin = {-145.52, -85.0977}, extent = {{-39.43, 24.94}, {20.1741, -8.82}}, textString = "dVREF")}));
end GENROE_AC7B;
