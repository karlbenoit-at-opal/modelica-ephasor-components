within OpalRT.GenUnits.GENROU;
class GENROU_REXSYS_IEEEG2_UEL1
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
  //REXSYS
  parameter Real TR_ex = 0.02 "voltage transducer time constant (sec) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real KVP_ex = 600 "voltage regulator proportional gain " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real KVI_ex = 0.5 "voltage regulator integral gain " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real VIMAX_ex = 0.2 "voltage regulator input limit (pu) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real TA_ex = 0.02 "voltage regulator time constant (sec) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real TB1_ex = 1 "lag-time constant (sec) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real TC1_ex = 10 "lead-time constant (sec) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real TB2_ex = 1.0 "lag-time constant (sec) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real TC2_ex = 1.0 "lead-time constant (sec) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real VRMAX_ex = 10 "maximum controller output (pu) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real VRMIN_ex = -10 "minimum controller output (pu) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real KF_ex = 0.045 "rate feedback gain " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real TF_ex = 5 "rate feedback >0 time constant (sec) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real TF1_ex = 1 "feedback lead-time constant (sec) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real TF2_ex = 1 "feedback lag-time constant (sec) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real FBF_ex = 1 "rate feedback signal flag " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real KIP_ex = 5.0 "field current regulator proportional gain " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real KII_ex = 0.5 "field current regulator integral gain " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real TP_ex = 0.5 "field current bridge time constant (sec) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real VFMAX_ex = 99 "maximum exciter field current (pu " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real VFMIN_ex = -99 "minimum exciter field current (pu) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real KH_ex = 0.5 "field voltage controller feedback gain " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real KE_ex = 0.4 "exciter field proportional constant" annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real TE_ex = 1.2 "exciter field time constant (sec >0)" annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real KC_ex = 0.5 "rectifier regulation factor (pu)" annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real KD_ex = 0.7 "exciter regulation factor (pu)" annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real E1_ex = 2.4 "exciter flux at knee of curve (pu) " annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real SE_E1_ex = 0.05 "saturation factor at knee" annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real E2_ex = 3.2 "maximum exciter (pu)" annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real SE_E2_ex = 0.3 "saturation factor at maximum flux" annotation(Dialog(tab = "REXSYS Parameters"));
  parameter Real F1IMF_ex = 0.5 "power supply limit factor" annotation(Dialog(tab = "REXSYS Parameters"));
  // UEL1 Parameters
  parameter Real KUR_uel = 2 "UEL radius setting (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real KUC_uel = 1 "(pu) UEL center setting (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real KUF_uel = 1 "(pu) UEL excitation system stabilizergain (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real VURMAX_uel = 1 "UEL maximum limit for radius phasor magnitude (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real VUCMAX_uel = 1 "UEL maximum limit for operating point phasor magnitude (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real KUI_uel = 1 "UEL integral gain (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real KUL_uel = 1 "UEL proportional gain (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real VUIMAX_uel = 1 "UEL integrator output maximum limit (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real VUIMIN_uel = 1 "UEL integrator output minimum limit (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real TU1_uel = 1 "UEL lead time constant (sec)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real TU2_uel = 1 "UEL lag time constant (sec)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real TU3_uel = 1 "UEL lead time constant (sec)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real TU4_uel = 1 "UEL lag time constant (sec)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real VULMAX_uel = 1 "UEL output maximum limit (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  parameter Real VULMIN_uel = 1 "UEL output minimum limit (pu)" annotation(Dialog(tab = "UEL1 Parameters"));
  //ieeeg2
  parameter String ID = "M1" "Machine Identifier" annotation(Dialog(tab = "IEEEG2"));
  parameter Real K_tg = 20 "K" annotation(Dialog(tab = "IEEEG2"));
  parameter Real T1_tg = 20 "T1" annotation(Dialog(tab = "IEEEG2"));
  parameter Real T2_tg = 20 "T2" annotation(Dialog(tab = "IEEEG2"));
  parameter Real T3_tg = 20 "T3(>0)" annotation(Dialog(tab = "IEEEG2"));
  parameter Real PMAX_tg = 20 "PMAX (pu on machine MVA rating)" annotation(Dialog(tab = "IEEEG2"));
  parameter Real PMIN_tg = 20 "PMIN (pu on machine MVA rating)" annotation(Dialog(tab = "IEEEG2"));
  parameter Real T4_tg = 20 "T3(>0)" annotation(Dialog(tab = "IEEEG2"));
  OpalRT.Electrical.Control.Excitation.REXSYS rexsys1(TR = TR_ex, KVP = KVP_ex, KVI = KVI_ex, VIMAX = VIMAX_ex, TA = TA_ex, TB1 = TB1_ex, TC1 = TC1_ex, TB2 = TB2_ex, TC2 = TC2_ex, VRMAX = VRMAX_ex, VRMIN = VRMIN_ex, KF = KF_ex, TF = TF_ex, TF1 = TF1_ex, TF2 = TF2_ex, FBF = FBF_ex, KIP = KIP_ex, KII = KII_ex, TP = TP_ex, VFMAX = VFMAX_ex, VFMIN = VFMIN_ex, KH = KH_ex, KE = KE_ex, TE = TE_ex, KC = KC_ex, KD = KD_ex, E1 = E1_ex, SE_E1 = SE_E1_ex, E2 = E2_ex, SE_E2 = SE_E2_ex, F1IMF = F1IMF_ex) annotation(Placement(visible = true, transformation(origin={-6,2}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  OpalRT.Electrical.Machine.SynchronousMachine.GENROU genrou1(
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
    S12 = S12) annotation(Placement(visible = true, transformation(origin = {60, -20}, extent = {{-25, -25}, {25, 25}}, rotation = 0)));
  Modelica.Blocks.Sources.Constant const(k = 0) annotation(Placement(visible = true, transformation(origin={-48,-12}, extent={{-4,-4},
            {4,4}}, rotation = 0)));
  OpalRT.NonElectrical.Connector.PwPin bus0 annotation(Placement(visible = true, transformation(origin={73,-55}, extent={{-5,-5}, {5,5}}, rotation = 0), iconTransformation(origin = {100, -80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.UnderExcitationLimiter.UEL1 uel11(
    KUR = KUR_uel,
    KUC = KUC_uel,
    KUF = KUF_uel,
    VURMAX = VURMAX_uel,
    VUCMAX = VUCMAX_uel,
    KUI = KUI_uel,
    KUL = KUL_uel,
    VUIMAX = VUIMAX_uel,
    VUIMIN = VUIMIN_uel,
    TU1 = TU1_uel,
    TU2 = TU2_uel,
    TU3 = TU3_uel,
    TU4 = TU4_uel,
    VULMAX = VULMAX_uel,
    VULMIN = VULMIN_uel) annotation(Placement(visible = true, transformation(origin={-72,0}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dVREF annotation(Placement(visible = true, transformation(origin={-51,-27}, extent={{-5,-5},
            {5,5}}, rotation = 0), iconTransformation(origin = {-100, -20}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  OpalRT.Electrical.Control.TurbineGovernor.IEEEG2 ieeeg21(
    K = K_tg,
    T1 = T1_tg,
    T2 = T2_tg,
    T3 = T3_tg,
    PMAX = PMAX_tg,
    PMIN = PMIN_tg,
    T4 = T4_tg) annotation(Placement(visible = true, transformation(origin={0,-48}, extent = {{-15, -15}, {15, 15}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin TRIP annotation(Placement(visible = true, transformation(origin={47,19}, extent = {{-5, -5}, {5, 5}}, rotation = 0), iconTransformation(origin={-100,80}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
  input OpalRT.NonElectrical.Connector.InputInterfacePin dGREF annotation(Placement(visible = true, transformation(origin={-51,-47}, extent={{-5,-5},
            {5,5}}, rotation = 0), iconTransformation(origin={-100,-60}, extent = {{-10, -10}, {10, 10}}, rotation = 0)));
equation 
  connect(bus0,genrou1.p) annotation(Line(points={{73,-55},{60,-55},{60,-45}}));
  connect(ieeeg21.PMECH, genrou1.PMECH) annotation (Line(points = {{15,-36},{20,
          -36},{20,-35},{35,-35}}, color = {0,0,127}));
  connect(genrou1.PMECH0, ieeeg21.PMECH0) annotation (Line(points={{35,-40},{24,
          -40},{24,-39},{15,-39}}, color = {0,0,127}));
  connect(genrou1.SLIP, ieeeg21.SLIP) annotation (Line(points={{85,-40},{90,-40},
          {90,-66},{-20,-66},{-20,-60},{-15,-60}}, color = {0,0,127}));
  connect(uel11.EX_AUX, genrou1.EX_AUX) annotation (Line(points={{-86.7,12},{-92,
          12},{-92,34},{30,34},{30,0},{34.5,0}}, color = {0,0,127}));
  connect(rexsys1.EFD0, genrou1.EFD0) annotation (Line(points = {{19,-18},{26,
          -18},{26,-20},{34.5,-20}}, color = {0,0,127}));
  connect(rexsys1.EFD, genrou1.EFD) annotation (Line(points = {{19,-13},{25.5,
          -13},{25.5,-13},{35,-13}}, color = {0,0,127}));
  connect(rexsys1.ETERM0, genrou1.ETERM0) annotation (Line(points = {{19,-8},{
          26,-8},{26,-7.5},{34.5,-7.5}}, color = {0,0,127}));
  connect(rexsys1.EX_AUX, genrou1.EX_AUX) annotation (Line(points = {{19,-1.5},
          {26.5,-1.5},{26.5,0},{34.5,0}}, color = {0,0,127}));
  connect(uel11.VUEL, rexsys1.VUEL) annotation (Line(points={{-56.4,0},{-46,0},{
          -46,7},{-31,7}}, color = {0,0,127}));
  connect(rexsys1.VOEL, const.y) annotation (Line(points={{-31,0},{-38,0},{
          -38,-12},{-43.6,-12}}, color={0,0,127}));
  connect(rexsys1.VOTHSG, const.y) annotation (Line(points={{-31,-7},{-38,-7},{-38,
          -12},{-43.6,-12}}, color = {0,0,127}));
  connect(uel11.VF, rexsys1.VF) annotation (Line(points={{-86.7,0},{-98,0},{-98,
          52},{26,52},{26,22},{20,22}}, color = {0,0,127}));
  connect(genrou1.XADIFD, rexsys1.XADIFD) annotation (Line(points={{85,-7.5},{90,
          -7.5},{90,30},{-40,30},{-40,14.5},{-31,14.5}}, color = {0,0,127}));
  connect(TRIP, genrou1.TRIP)
    annotation (Line(points = {{47,19},{60,19},{60,5}}, color={0,0,0}));
  connect(genrou1.MBASE, ieeeg21.MBASE) annotation (Line(points={{85,-33.5},{92,
          -33.5},{92,-68},{-22,-68},{-22,-54},{-15,-54}}, color={0,0,127}));
  connect(genrou1.VI, ieeeg21.VI) annotation (Line(points={{85,-20},{96,-20},{96,
          -72},{-26,-72},{-26,-48},{-15,-48}}, color={0,0,127}));
  connect(genrou1.VI, uel11.VI) annotation (Line(points={{85,-20},{96,-20},{96,40},
          {-94,40},{-94,6},{-86.7,6}}, color={0,0,127}));
  connect(dVREF, rexsys1.dVREF) annotation (Line(points={{-51,-27},{-36,-27},{-36,
          -13},{-31,-13}}, color={0,0,0}));
  connect(dGREF, ieeeg21.dGREF) annotation (Line(points={{-51,-47},{-40,-47},{-40,
          -36},{-15,-36}}, color={0,0,0}));
  connect(genrou1.VI, rexsys1.VI) annotation (Line(points={{85,-20},{96,-20},{96,
          34},{36,34},{36,17},{19,17}}, color={0,0,127}));
  annotation(Diagram(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2})), Icon(coordinateSystem(extent = {{-100, -100}, {100, 100}}, preserveAspectRatio = true, initialScale = 0.1, grid = {2, 2}), graphics={ Text(origin = {-81.55, 7.17}, extent = {{-12.53, 6.49}, {175.86, -14.01}}, textString = "GENROU_ESAC5A_IEEEG2_UEL1"), Rectangle(origin = {0.113895, -0.341686}, extent = {{-99.8861, 99.8861}, {99.8861, -99.8861}})}));
end GENROU_REXSYS_IEEEG2_UEL1;
