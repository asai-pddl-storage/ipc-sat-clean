(define (problem os-sequencedstrips-p230_1)
(:domain openstacks-sequencedstrips-nonADL-nonNegated)
(:objects 
n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12 n13 n14 n15 n16 n17 n18 n19 n20 n21 n22 n23 n24 n25 n26 n27 n28 n29 n30 n31 n32 n33 n34 n35 n36 n37 n38 n39 n40 n41 n42 n43 n44 n45 n46 n47 n48 n49 n50 n51 n52 n53 n54 n55 n56 n57 n58 n59 n60 n61 n62 n63 n64 n65 n66 n67 n68 n69 n70 n71 n72 n73 n74 n75 n76 n77 n78 n79 n80 n81 n82 n83 n84 n85 n86 n87 n88 n89 n90 n91 n92 n93 n94 n95 n96 n97 n98 n99 n100 n101 n102 n103 n104 n105 n106 n107 n108 n109 n110 n111 n112 n113 n114 n115 n116 n117 n118 n119 n120 n121 n122 n123 n124 n125 n126 n127 n128 n129 n130 n131 n132 n133 n134 n135 n136 n137 n138 n139 n140 n141 n142 n143 n144 n145 n146 n147 n148 n149 n150 n151 n152 n153 n154 n155 n156 n157 n158 n159 n160 n161 n162 n163 n164 n165 n166 n167 n168 n169 n170 n171 n172 n173 n174 n175 n176 n177 n178 n179 n180 n181 n182 n183 n184 n185 n186 n187 n188 n189 n190 n191 n192 n193 n194 n195 n196 n197 n198 n199 n200 n201 n202 n203 n204 n205 n206 n207 n208 n209 n210 n211 n212 n213 n214 n215 n216 n217 n218 n219 n220 n221 n222 n223 n224 n225 n226 n227 n228 n229 n230  - count
)

(:init
(next-count n0 n1) (next-count n1 n2) (next-count n2 n3) (next-count n3 n4) (next-count n4 n5) (next-count n5 n6) (next-count n6 n7) (next-count n7 n8) (next-count n8 n9) (next-count n9 n10) (next-count n10 n11) (next-count n11 n12) (next-count n12 n13) (next-count n13 n14) (next-count n14 n15) (next-count n15 n16) (next-count n16 n17) (next-count n17 n18) (next-count n18 n19) (next-count n19 n20) (next-count n20 n21) (next-count n21 n22) (next-count n22 n23) (next-count n23 n24) (next-count n24 n25) (next-count n25 n26) (next-count n26 n27) (next-count n27 n28) (next-count n28 n29) (next-count n29 n30) (next-count n30 n31) (next-count n31 n32) (next-count n32 n33) (next-count n33 n34) (next-count n34 n35) (next-count n35 n36) (next-count n36 n37) (next-count n37 n38) (next-count n38 n39) (next-count n39 n40) (next-count n40 n41) (next-count n41 n42) (next-count n42 n43) (next-count n43 n44) (next-count n44 n45) (next-count n45 n46) (next-count n46 n47) (next-count n47 n48) (next-count n48 n49) (next-count n49 n50) (next-count n50 n51) (next-count n51 n52) (next-count n52 n53) (next-count n53 n54) (next-count n54 n55) (next-count n55 n56) (next-count n56 n57) (next-count n57 n58) (next-count n58 n59) (next-count n59 n60) (next-count n60 n61) (next-count n61 n62) (next-count n62 n63) (next-count n63 n64) (next-count n64 n65) (next-count n65 n66) (next-count n66 n67) (next-count n67 n68) (next-count n68 n69) (next-count n69 n70) (next-count n70 n71) (next-count n71 n72) (next-count n72 n73) (next-count n73 n74) (next-count n74 n75) (next-count n75 n76) (next-count n76 n77) (next-count n77 n78) (next-count n78 n79) (next-count n79 n80) (next-count n80 n81) (next-count n81 n82) (next-count n82 n83) (next-count n83 n84) (next-count n84 n85) (next-count n85 n86) (next-count n86 n87) (next-count n87 n88) (next-count n88 n89) (next-count n89 n90) (next-count n90 n91) (next-count n91 n92) (next-count n92 n93) (next-count n93 n94) (next-count n94 n95) (next-count n95 n96) (next-count n96 n97) (next-count n97 n98) (next-count n98 n99) (next-count n99 n100) (next-count n100 n101) (next-count n101 n102) (next-count n102 n103) (next-count n103 n104) (next-count n104 n105) (next-count n105 n106) (next-count n106 n107) (next-count n107 n108) (next-count n108 n109) (next-count n109 n110) (next-count n110 n111) (next-count n111 n112) (next-count n112 n113) (next-count n113 n114) (next-count n114 n115) (next-count n115 n116) (next-count n116 n117) (next-count n117 n118) (next-count n118 n119) (next-count n119 n120) (next-count n120 n121) (next-count n121 n122) (next-count n122 n123) (next-count n123 n124) (next-count n124 n125) (next-count n125 n126) (next-count n126 n127) (next-count n127 n128) (next-count n128 n129) (next-count n129 n130) (next-count n130 n131) (next-count n131 n132) (next-count n132 n133) (next-count n133 n134) (next-count n134 n135) (next-count n135 n136) (next-count n136 n137) (next-count n137 n138) (next-count n138 n139) (next-count n139 n140) (next-count n140 n141) (next-count n141 n142) (next-count n142 n143) (next-count n143 n144) (next-count n144 n145) (next-count n145 n146) (next-count n146 n147) (next-count n147 n148) (next-count n148 n149) (next-count n149 n150) (next-count n150 n151) (next-count n151 n152) (next-count n152 n153) (next-count n153 n154) (next-count n154 n155) (next-count n155 n156) (next-count n156 n157) (next-count n157 n158) (next-count n158 n159) (next-count n159 n160) (next-count n160 n161) (next-count n161 n162) (next-count n162 n163) (next-count n163 n164) (next-count n164 n165) (next-count n165 n166) (next-count n166 n167) (next-count n167 n168) (next-count n168 n169) (next-count n169 n170) (next-count n170 n171) (next-count n171 n172) (next-count n172 n173) (next-count n173 n174) (next-count n174 n175) (next-count n175 n176) (next-count n176 n177) (next-count n177 n178) (next-count n178 n179) (next-count n179 n180) (next-count n180 n181) (next-count n181 n182) (next-count n182 n183) (next-count n183 n184) (next-count n184 n185) (next-count n185 n186) (next-count n186 n187) (next-count n187 n188) (next-count n188 n189) (next-count n189 n190) (next-count n190 n191) (next-count n191 n192) (next-count n192 n193) (next-count n193 n194) (next-count n194 n195) (next-count n195 n196) (next-count n196 n197) (next-count n197 n198) (next-count n198 n199) (next-count n199 n200) (next-count n200 n201) (next-count n201 n202) (next-count n202 n203) (next-count n203 n204) (next-count n204 n205) (next-count n205 n206) (next-count n206 n207) (next-count n207 n208) (next-count n208 n209) (next-count n209 n210) (next-count n210 n211) (next-count n211 n212) (next-count n212 n213) (next-count n213 n214) (next-count n214 n215) (next-count n215 n216) (next-count n216 n217) (next-count n217 n218) (next-count n218 n219) (next-count n219 n220) (next-count n220 n221) (next-count n221 n222) (next-count n222 n223) (next-count n223 n224) (next-count n224 n225) (next-count n225 n226) (next-count n226 n227) (next-count n227 n228) (next-count n228 n229) (next-count n229 n230) 
(stacks-avail n0)

(waiting o1)
(includes o1 p30)(includes o1 p45)(includes o1 p112)(includes o1 p115)(includes o1 p136)(includes o1 p167)(includes o1 p194)(includes o1 p211)

(waiting o2)
(includes o2 p12)(includes o2 p131)(includes o2 p144)(includes o2 p150)(includes o2 p159)(includes o2 p222)(includes o2 p223)

(waiting o3)
(includes o3 p13)(includes o3 p27)(includes o3 p32)(includes o3 p95)(includes o3 p169)(includes o3 p200)

(waiting o4)
(includes o4 p27)(includes o4 p100)(includes o4 p120)(includes o4 p169)(includes o4 p201)

(waiting o5)
(includes o5 p28)(includes o5 p50)(includes o5 p71)(includes o5 p91)(includes o5 p118)(includes o5 p152)(includes o5 p219)

(waiting o6)
(includes o6 p79)(includes o6 p94)(includes o6 p96)(includes o6 p116)(includes o6 p127)(includes o6 p152)(includes o6 p158)(includes o6 p180)(includes o6 p217)

(waiting o7)
(includes o7 p7)(includes o7 p65)(includes o7 p150)(includes o7 p152)(includes o7 p216)

(waiting o8)
(includes o8 p5)(includes o8 p6)(includes o8 p50)(includes o8 p67)(includes o8 p80)(includes o8 p87)(includes o8 p102)(includes o8 p106)(includes o8 p149)(includes o8 p175)(includes o8 p227)

(waiting o9)
(includes o9 p94)(includes o9 p171)(includes o9 p187)(includes o9 p229)

(waiting o10)
(includes o10 p28)(includes o10 p41)(includes o10 p114)(includes o10 p174)(includes o10 p199)(includes o10 p206)

(waiting o11)
(includes o11 p12)(includes o11 p160)(includes o11 p172)(includes o11 p193)(includes o11 p196)(includes o11 p202)

(waiting o12)
(includes o12 p39)(includes o12 p57)(includes o12 p104)(includes o12 p116)(includes o12 p148)(includes o12 p166)

(waiting o13)
(includes o13 p8)(includes o13 p16)(includes o13 p69)(includes o13 p80)(includes o13 p192)(includes o13 p203)(includes o13 p211)

(waiting o14)
(includes o14 p27)(includes o14 p31)(includes o14 p37)(includes o14 p89)(includes o14 p95)(includes o14 p137)(includes o14 p138)(includes o14 p160)(includes o14 p215)

(waiting o15)
(includes o15 p13)(includes o15 p27)(includes o15 p57)(includes o15 p78)(includes o15 p79)(includes o15 p101)(includes o15 p108)(includes o15 p115)

(waiting o16)
(includes o16 p22)(includes o16 p33)(includes o16 p58)(includes o16 p125)(includes o16 p142)(includes o16 p172)(includes o16 p183)(includes o16 p189)(includes o16 p190)(includes o16 p200)(includes o16 p215)

(waiting o17)
(includes o17 p43)(includes o17 p65)(includes o17 p72)(includes o17 p78)(includes o17 p82)(includes o17 p98)(includes o17 p126)(includes o17 p183)(includes o17 p206)(includes o17 p208)

(waiting o18)
(includes o18 p76)(includes o18 p93)(includes o18 p210)(includes o18 p219)

(waiting o19)
(includes o19 p26)(includes o19 p36)(includes o19 p59)(includes o19 p96)(includes o19 p108)(includes o19 p116)(includes o19 p145)(includes o19 p171)(includes o19 p188)(includes o19 p197)(includes o19 p200)

(waiting o20)
(includes o20 p35)(includes o20 p52)(includes o20 p56)(includes o20 p150)

(waiting o21)
(includes o21 p54)(includes o21 p88)(includes o21 p133)(includes o21 p229)

(waiting o22)
(includes o22 p42)(includes o22 p151)(includes o22 p171)(includes o22 p188)(includes o22 p197)

(waiting o23)
(includes o23 p10)(includes o23 p95)(includes o23 p146)

(waiting o24)
(includes o24 p62)(includes o24 p105)(includes o24 p129)(includes o24 p155)(includes o24 p160)

(waiting o25)
(includes o25 p59)(includes o25 p88)(includes o25 p174)

(waiting o26)
(includes o26 p55)(includes o26 p89)(includes o26 p116)(includes o26 p177)(includes o26 p191)(includes o26 p196)(includes o26 p205)

(waiting o27)
(includes o27 p69)(includes o27 p87)(includes o27 p96)(includes o27 p171)

(waiting o28)
(includes o28 p2)(includes o28 p27)(includes o28 p43)(includes o28 p92)(includes o28 p100)(includes o28 p112)(includes o28 p127)(includes o28 p134)(includes o28 p166)(includes o28 p173)(includes o28 p199)

(waiting o29)
(includes o29 p103)(includes o29 p142)

(waiting o30)
(includes o30 p85)(includes o30 p145)(includes o30 p147)(includes o30 p192)(includes o30 p208)(includes o30 p212)(includes o30 p218)

(waiting o31)
(includes o31 p24)(includes o31 p43)(includes o31 p112)(includes o31 p130)(includes o31 p163)(includes o31 p226)

(waiting o32)
(includes o32 p44)(includes o32 p92)(includes o32 p99)(includes o32 p130)(includes o32 p135)(includes o32 p141)(includes o32 p200)(includes o32 p220)

(waiting o33)
(includes o33 p29)(includes o33 p108)(includes o33 p140)(includes o33 p158)(includes o33 p205)(includes o33 p219)

(waiting o34)
(includes o34 p27)(includes o34 p40)(includes o34 p43)(includes o34 p48)(includes o34 p115)(includes o34 p124)(includes o34 p146)(includes o34 p156)(includes o34 p161)(includes o34 p208)(includes o34 p218)

(waiting o35)
(includes o35 p23)(includes o35 p36)(includes o35 p71)(includes o35 p83)(includes o35 p86)(includes o35 p120)(includes o35 p153)(includes o35 p176)(includes o35 p177)(includes o35 p200)

(waiting o36)
(includes o36 p8)(includes o36 p12)(includes o36 p32)(includes o36 p56)(includes o36 p100)(includes o36 p177)(includes o36 p199)

(waiting o37)
(includes o37 p43)(includes o37 p94)(includes o37 p118)

(waiting o38)
(includes o38 p94)(includes o38 p128)(includes o38 p174)

(waiting o39)
(includes o39 p14)(includes o39 p73)(includes o39 p109)(includes o39 p173)(includes o39 p207)

(waiting o40)
(includes o40 p129)(includes o40 p161)(includes o40 p216)(includes o40 p224)

(waiting o41)
(includes o41 p28)(includes o41 p52)(includes o41 p65)(includes o41 p83)(includes o41 p119)(includes o41 p142)(includes o41 p213)(includes o41 p221)

(waiting o42)
(includes o42 p13)(includes o42 p150)(includes o42 p156)(includes o42 p160)(includes o42 p168)(includes o42 p211)(includes o42 p214)

(waiting o43)
(includes o43 p16)(includes o43 p81)(includes o43 p115)(includes o43 p121)(includes o43 p148)(includes o43 p150)(includes o43 p161)(includes o43 p192)(includes o43 p193)(includes o43 p228)

(waiting o44)
(includes o44 p71)(includes o44 p74)(includes o44 p102)(includes o44 p110)(includes o44 p116)(includes o44 p122)(includes o44 p132)(includes o44 p176)

(waiting o45)
(includes o45 p4)(includes o45 p8)(includes o45 p36)(includes o45 p58)(includes o45 p72)(includes o45 p93)(includes o45 p108)(includes o45 p135)(includes o45 p172)(includes o45 p221)

(waiting o46)
(includes o46 p41)(includes o46 p48)(includes o46 p76)(includes o46 p117)(includes o46 p130)(includes o46 p174)

(waiting o47)
(includes o47 p6)(includes o47 p12)(includes o47 p92)(includes o47 p203)

(waiting o48)
(includes o48 p55)(includes o48 p58)(includes o48 p78)(includes o48 p92)(includes o48 p99)(includes o48 p106)(includes o48 p161)(includes o48 p229)

(waiting o49)
(includes o49 p28)(includes o49 p56)(includes o49 p63)(includes o49 p172)

(waiting o50)
(includes o50 p97)(includes o50 p108)(includes o50 p190)(includes o50 p213)(includes o50 p228)

(waiting o51)
(includes o51 p4)(includes o51 p21)(includes o51 p37)(includes o51 p81)(includes o51 p99)(includes o51 p105)(includes o51 p190)

(waiting o52)
(includes o52 p35)(includes o52 p41)(includes o52 p125)(includes o52 p151)(includes o52 p205)

(waiting o53)
(includes o53 p71)(includes o53 p76)(includes o53 p132)

(waiting o54)
(includes o54 p96)(includes o54 p118)(includes o54 p145)(includes o54 p148)(includes o54 p168)

(waiting o55)
(includes o55 p1)(includes o55 p23)(includes o55 p105)(includes o55 p111)(includes o55 p124)(includes o55 p161)

(waiting o56)
(includes o56 p29)(includes o56 p173)(includes o56 p208)

(waiting o57)
(includes o57 p128)(includes o57 p142)(includes o57 p143)(includes o57 p221)(includes o57 p223)

(waiting o58)
(includes o58 p48)(includes o58 p52)(includes o58 p76)(includes o58 p97)(includes o58 p99)(includes o58 p148)(includes o58 p151)(includes o58 p169)(includes o58 p191)(includes o58 p197)(includes o58 p225)

(waiting o59)
(includes o59 p32)(includes o59 p35)(includes o59 p70)(includes o59 p76)(includes o59 p111)(includes o59 p123)(includes o59 p209)

(waiting o60)
(includes o60 p1)(includes o60 p56)(includes o60 p78)(includes o60 p82)(includes o60 p94)(includes o60 p108)(includes o60 p137)

(waiting o61)
(includes o61 p3)(includes o61 p89)(includes o61 p145)(includes o61 p171)(includes o61 p204)(includes o61 p213)

(waiting o62)
(includes o62 p68)(includes o62 p113)(includes o62 p123)

(waiting o63)
(includes o63 p3)(includes o63 p20)(includes o63 p45)(includes o63 p81)(includes o63 p116)(includes o63 p138)(includes o63 p152)(includes o63 p197)(includes o63 p207)

(waiting o64)
(includes o64 p17)(includes o64 p29)(includes o64 p93)(includes o64 p96)(includes o64 p98)(includes o64 p192)(includes o64 p220)

(waiting o65)
(includes o65 p56)(includes o65 p100)(includes o65 p155)(includes o65 p163)(includes o65 p226)

(waiting o66)
(includes o66 p106)(includes o66 p142)(includes o66 p160)(includes o66 p175)(includes o66 p188)(includes o66 p193)

(waiting o67)
(includes o67 p13)(includes o67 p19)(includes o67 p23)(includes o67 p138)(includes o67 p140)(includes o67 p163)(includes o67 p191)

(waiting o68)
(includes o68 p1)(includes o68 p7)(includes o68 p40)(includes o68 p73)(includes o68 p105)(includes o68 p191)(includes o68 p205)

(waiting o69)
(includes o69 p25)(includes o69 p77)(includes o69 p87)(includes o69 p158)(includes o69 p229)

(waiting o70)
(includes o70 p47)(includes o70 p110)(includes o70 p137)(includes o70 p183)(includes o70 p197)

(waiting o71)
(includes o71 p60)(includes o71 p149)(includes o71 p175)

(waiting o72)
(includes o72 p32)(includes o72 p110)(includes o72 p144)(includes o72 p171)

(waiting o73)
(includes o73 p36)(includes o73 p85)(includes o73 p94)(includes o73 p143)(includes o73 p149)(includes o73 p217)

(waiting o74)
(includes o74 p18)(includes o74 p27)(includes o74 p78)(includes o74 p89)(includes o74 p104)(includes o74 p160)(includes o74 p198)

(waiting o75)
(includes o75 p31)(includes o75 p49)(includes o75 p121)(includes o75 p184)

(waiting o76)
(includes o76 p2)(includes o76 p29)(includes o76 p33)(includes o76 p99)(includes o76 p155)(includes o76 p230)

(waiting o77)
(includes o77 p41)(includes o77 p46)(includes o77 p85)(includes o77 p124)(includes o77 p147)(includes o77 p195)

(waiting o78)
(includes o78 p1)(includes o78 p6)(includes o78 p15)(includes o78 p34)(includes o78 p51)(includes o78 p75)(includes o78 p87)(includes o78 p111)(includes o78 p139)

(waiting o79)
(includes o79 p166)

(waiting o80)
(includes o80 p36)(includes o80 p100)(includes o80 p109)(includes o80 p146)

(waiting o81)
(includes o81 p61)(includes o81 p145)(includes o81 p152)(includes o81 p191)(includes o81 p216)(includes o81 p229)

(waiting o82)
(includes o82 p49)(includes o82 p94)

(waiting o83)
(includes o83 p70)(includes o83 p98)(includes o83 p206)

(waiting o84)
(includes o84 p197)(includes o84 p215)

(waiting o85)
(includes o85 p36)(includes o85 p98)(includes o85 p99)(includes o85 p116)(includes o85 p118)(includes o85 p144)(includes o85 p161)(includes o85 p176)

(waiting o86)
(includes o86 p3)(includes o86 p9)(includes o86 p79)(includes o86 p81)(includes o86 p196)(includes o86 p198)

(waiting o87)
(includes o87 p56)(includes o87 p85)(includes o87 p137)(includes o87 p198)

(waiting o88)
(includes o88 p49)(includes o88 p65)(includes o88 p103)(includes o88 p118)

(waiting o89)
(includes o89 p22)(includes o89 p65)(includes o89 p136)(includes o89 p177)(includes o89 p192)(includes o89 p210)(includes o89 p222)

(waiting o90)
(includes o90 p15)(includes o90 p27)(includes o90 p90)(includes o90 p91)(includes o90 p111)(includes o90 p141)(includes o90 p160)(includes o90 p197)

(waiting o91)
(includes o91 p111)(includes o91 p125)(includes o91 p145)(includes o91 p181)(includes o91 p207)

(waiting o92)
(includes o92 p8)(includes o92 p23)(includes o92 p42)(includes o92 p89)(includes o92 p108)(includes o92 p117)(includes o92 p157)(includes o92 p181)(includes o92 p223)

(waiting o93)
(includes o93 p54)(includes o93 p78)(includes o93 p90)(includes o93 p117)(includes o93 p202)(includes o93 p212)

(waiting o94)
(includes o94 p73)(includes o94 p102)(includes o94 p104)(includes o94 p126)(includes o94 p219)(includes o94 p222)

(waiting o95)
(includes o95 p3)(includes o95 p58)(includes o95 p59)(includes o95 p67)(includes o95 p108)(includes o95 p114)(includes o95 p131)(includes o95 p166)(includes o95 p184)(includes o95 p211)

(waiting o96)
(includes o96 p13)(includes o96 p14)(includes o96 p35)(includes o96 p101)(includes o96 p106)(includes o96 p171)

(waiting o97)
(includes o97 p19)(includes o97 p99)(includes o97 p126)(includes o97 p168)(includes o97 p201)

(waiting o98)
(includes o98 p41)(includes o98 p110)(includes o98 p129)(includes o98 p134)

(waiting o99)
(includes o99 p117)(includes o99 p155)(includes o99 p157)

(waiting o100)
(includes o100 p24)(includes o100 p105)(includes o100 p180)(includes o100 p213)

(waiting o101)
(includes o101 p24)(includes o101 p169)

(waiting o102)
(includes o102 p60)(includes o102 p74)(includes o102 p223)

(waiting o103)
(includes o103 p11)(includes o103 p17)(includes o103 p33)(includes o103 p97)(includes o103 p162)(includes o103 p165)(includes o103 p168)(includes o103 p186)

(waiting o104)
(includes o104 p33)(includes o104 p44)(includes o104 p66)(includes o104 p114)(includes o104 p138)(includes o104 p182)

(waiting o105)
(includes o105 p14)(includes o105 p68)(includes o105 p86)(includes o105 p88)(includes o105 p162)(includes o105 p171)(includes o105 p179)(includes o105 p199)

(waiting o106)
(includes o106 p10)(includes o106 p91)(includes o106 p124)(includes o106 p162)(includes o106 p230)

(waiting o107)
(includes o107 p5)(includes o107 p18)(includes o107 p38)(includes o107 p155)

(waiting o108)
(includes o108 p93)(includes o108 p101)(includes o108 p149)

(waiting o109)
(includes o109 p6)(includes o109 p17)(includes o109 p45)(includes o109 p56)(includes o109 p87)(includes o109 p105)(includes o109 p152)

(waiting o110)
(includes o110 p71)(includes o110 p84)(includes o110 p93)(includes o110 p95)(includes o110 p164)(includes o110 p177)

(waiting o111)
(includes o111 p19)(includes o111 p99)(includes o111 p158)(includes o111 p179)(includes o111 p182)(includes o111 p215)(includes o111 p226)

(waiting o112)
(includes o112 p64)(includes o112 p85)(includes o112 p109)(includes o112 p121)(includes o112 p130)(includes o112 p149)(includes o112 p188)(includes o112 p197)(includes o112 p228)

(waiting o113)
(includes o113 p4)(includes o113 p34)(includes o113 p40)(includes o113 p46)(includes o113 p81)(includes o113 p111)(includes o113 p122)(includes o113 p141)(includes o113 p149)(includes o113 p153)(includes o113 p158)

(waiting o114)
(includes o114 p5)(includes o114 p25)(includes o114 p56)(includes o114 p66)(includes o114 p133)(includes o114 p160)

(waiting o115)
(includes o115 p12)(includes o115 p17)(includes o115 p26)(includes o115 p89)(includes o115 p153)(includes o115 p180)(includes o115 p204)

(waiting o116)
(includes o116 p153)(includes o116 p164)(includes o116 p167)(includes o116 p185)

(waiting o117)
(includes o117 p8)(includes o117 p176)

(waiting o118)
(includes o118 p2)(includes o118 p100)(includes o118 p133)(includes o118 p175)(includes o118 p176)(includes o118 p186)

(waiting o119)
(includes o119 p31)(includes o119 p36)(includes o119 p41)

(waiting o120)
(includes o120 p52)(includes o120 p97)(includes o120 p119)(includes o120 p126)(includes o120 p199)(includes o120 p228)

(waiting o121)
(includes o121 p10)(includes o121 p46)(includes o121 p58)(includes o121 p98)(includes o121 p107)(includes o121 p127)(includes o121 p174)(includes o121 p199)

(waiting o122)
(includes o122 p44)(includes o122 p48)(includes o122 p123)

(waiting o123)
(includes o123 p5)(includes o123 p40)(includes o123 p56)(includes o123 p69)(includes o123 p110)(includes o123 p199)

(waiting o124)
(includes o124 p65)

(waiting o125)
(includes o125 p138)(includes o125 p141)(includes o125 p144)(includes o125 p205)

(waiting o126)
(includes o126 p7)(includes o126 p10)(includes o126 p45)(includes o126 p65)(includes o126 p150)

(waiting o127)
(includes o127 p34)(includes o127 p68)(includes o127 p113)(includes o127 p119)(includes o127 p138)(includes o127 p203)

(waiting o128)
(includes o128 p120)(includes o128 p121)(includes o128 p126)(includes o128 p135)(includes o128 p224)

(waiting o129)
(includes o129 p36)(includes o129 p37)(includes o129 p48)(includes o129 p49)(includes o129 p121)(includes o129 p159)(includes o129 p196)(includes o129 p228)

(waiting o130)
(includes o130 p34)(includes o130 p153)(includes o130 p168)(includes o130 p196)

(waiting o131)
(includes o131 p6)(includes o131 p45)(includes o131 p67)(includes o131 p93)(includes o131 p103)(includes o131 p109)(includes o131 p221)

(waiting o132)
(includes o132 p72)(includes o132 p156)(includes o132 p159)(includes o132 p169)(includes o132 p181)(includes o132 p217)(includes o132 p221)

(waiting o133)
(includes o133 p65)(includes o133 p132)(includes o133 p158)(includes o133 p171)(includes o133 p190)(includes o133 p204)(includes o133 p220)

(waiting o134)
(includes o134 p10)(includes o134 p77)(includes o134 p205)(includes o134 p225)

(waiting o135)
(includes o135 p9)(includes o135 p34)(includes o135 p141)(includes o135 p197)(includes o135 p202)

(waiting o136)
(includes o136 p68)(includes o136 p210)

(waiting o137)
(includes o137 p11)(includes o137 p14)(includes o137 p27)(includes o137 p86)(includes o137 p144)(includes o137 p162)(includes o137 p169)(includes o137 p176)(includes o137 p196)

(waiting o138)
(includes o138 p3)(includes o138 p5)(includes o138 p19)(includes o138 p36)(includes o138 p58)(includes o138 p82)(includes o138 p91)(includes o138 p126)(includes o138 p129)(includes o138 p178)(includes o138 p203)

(waiting o139)
(includes o139 p4)(includes o139 p127)(includes o139 p137)(includes o139 p221)

(waiting o140)
(includes o140 p14)(includes o140 p23)(includes o140 p50)(includes o140 p66)(includes o140 p98)(includes o140 p100)(includes o140 p117)(includes o140 p157)(includes o140 p216)

(waiting o141)
(includes o141 p118)

(waiting o142)
(includes o142 p19)(includes o142 p22)(includes o142 p28)(includes o142 p52)(includes o142 p91)(includes o142 p149)(includes o142 p168)(includes o142 p216)(includes o142 p229)

(waiting o143)
(includes o143 p47)(includes o143 p80)(includes o143 p120)(includes o143 p138)

(waiting o144)
(includes o144 p30)(includes o144 p43)(includes o144 p83)(includes o144 p140)(includes o144 p150)(includes o144 p163)(includes o144 p216)(includes o144 p222)

(waiting o145)
(includes o145 p33)(includes o145 p54)(includes o145 p80)(includes o145 p81)(includes o145 p225)

(waiting o146)
(includes o146 p12)(includes o146 p18)(includes o146 p72)(includes o146 p86)(includes o146 p114)(includes o146 p127)(includes o146 p142)(includes o146 p168)(includes o146 p186)(includes o146 p221)

(waiting o147)
(includes o147 p27)(includes o147 p56)(includes o147 p68)(includes o147 p112)

(waiting o148)
(includes o148 p21)(includes o148 p76)(includes o148 p110)(includes o148 p120)(includes o148 p171)(includes o148 p192)(includes o148 p218)(includes o148 p228)

(waiting o149)
(includes o149 p20)(includes o149 p61)(includes o149 p143)

(waiting o150)
(includes o150 p93)(includes o150 p96)(includes o150 p118)(includes o150 p138)

(waiting o151)
(includes o151 p3)(includes o151 p42)(includes o151 p78)(includes o151 p95)(includes o151 p100)(includes o151 p157)(includes o151 p214)

(waiting o152)
(includes o152 p4)(includes o152 p17)(includes o152 p82)(includes o152 p114)(includes o152 p143)(includes o152 p203)

(waiting o153)
(includes o153 p21)(includes o153 p43)(includes o153 p155)(includes o153 p185)

(waiting o154)
(includes o154 p75)(includes o154 p95)(includes o154 p138)(includes o154 p174)(includes o154 p206)

(waiting o155)
(includes o155 p10)(includes o155 p41)(includes o155 p86)(includes o155 p131)(includes o155 p201)

(waiting o156)
(includes o156 p21)(includes o156 p33)(includes o156 p79)(includes o156 p183)

(waiting o157)
(includes o157 p15)(includes o157 p20)(includes o157 p52)(includes o157 p55)(includes o157 p58)(includes o157 p64)(includes o157 p67)(includes o157 p81)(includes o157 p100)(includes o157 p106)(includes o157 p127)(includes o157 p160)

(waiting o158)
(includes o158 p20)(includes o158 p134)(includes o158 p157)(includes o158 p176)(includes o158 p191)(includes o158 p193)

(waiting o159)
(includes o159 p74)(includes o159 p131)

(waiting o160)
(includes o160 p56)(includes o160 p58)(includes o160 p79)(includes o160 p116)(includes o160 p125)

(waiting o161)
(includes o161 p5)(includes o161 p34)(includes o161 p39)(includes o161 p84)(includes o161 p155)(includes o161 p182)

(waiting o162)
(includes o162 p42)(includes o162 p60)(includes o162 p97)(includes o162 p125)(includes o162 p128)(includes o162 p149)(includes o162 p156)(includes o162 p182)

(waiting o163)
(includes o163 p97)(includes o163 p128)(includes o163 p186)(includes o163 p187)

(waiting o164)
(includes o164 p47)(includes o164 p141)(includes o164 p188)(includes o164 p218)

(waiting o165)
(includes o165 p116)(includes o165 p193)

(waiting o166)
(includes o166 p33)(includes o166 p42)(includes o166 p74)(includes o166 p88)(includes o166 p175)(includes o166 p190)

(waiting o167)
(includes o167 p21)(includes o167 p26)(includes o167 p59)(includes o167 p64)(includes o167 p91)(includes o167 p101)(includes o167 p115)

(waiting o168)
(includes o168 p9)(includes o168 p32)(includes o168 p128)(includes o168 p143)(includes o168 p169)

(waiting o169)
(includes o169 p31)(includes o169 p45)(includes o169 p60)(includes o169 p164)

(waiting o170)
(includes o170 p49)(includes o170 p76)(includes o170 p80)(includes o170 p131)(includes o170 p220)

(waiting o171)
(includes o171 p37)(includes o171 p111)(includes o171 p119)(includes o171 p127)(includes o171 p170)(includes o171 p184)(includes o171 p213)

(waiting o172)
(includes o172 p21)(includes o172 p32)(includes o172 p46)(includes o172 p87)(includes o172 p103)(includes o172 p111)(includes o172 p148)(includes o172 p196)

(waiting o173)
(includes o173 p4)(includes o173 p62)(includes o173 p131)(includes o173 p147)(includes o173 p163)(includes o173 p201)(includes o173 p222)

(waiting o174)
(includes o174 p31)(includes o174 p126)(includes o174 p198)

(waiting o175)
(includes o175 p26)(includes o175 p153)

(waiting o176)
(includes o176 p5)(includes o176 p48)(includes o176 p86)(includes o176 p129)(includes o176 p150)(includes o176 p185)

(waiting o177)
(includes o177 p25)(includes o177 p102)(includes o177 p153)(includes o177 p201)

(waiting o178)
(includes o178 p12)(includes o178 p51)(includes o178 p73)(includes o178 p103)(includes o178 p118)(includes o178 p127)(includes o178 p154)(includes o178 p168)(includes o178 p174)(includes o178 p190)(includes o178 p207)

(waiting o179)
(includes o179 p28)(includes o179 p62)(includes o179 p117)(includes o179 p128)(includes o179 p163)(includes o179 p177)(includes o179 p215)

(waiting o180)
(includes o180 p197)(includes o180 p219)

(waiting o181)
(includes o181 p45)(includes o181 p57)(includes o181 p85)(includes o181 p101)(includes o181 p103)(includes o181 p203)(includes o181 p218)

(waiting o182)
(includes o182 p32)(includes o182 p38)(includes o182 p146)(includes o182 p217)(includes o182 p225)(includes o182 p228)

(waiting o183)
(includes o183 p1)(includes o183 p44)(includes o183 p52)(includes o183 p80)(includes o183 p100)(includes o183 p109)(includes o183 p181)(includes o183 p186)

(waiting o184)
(includes o184 p19)(includes o184 p138)(includes o184 p157)(includes o184 p213)

(waiting o185)
(includes o185 p47)(includes o185 p65)(includes o185 p107)(includes o185 p175)(includes o185 p202)(includes o185 p230)

(waiting o186)
(includes o186 p9)(includes o186 p122)(includes o186 p203)(includes o186 p221)

(waiting o187)
(includes o187 p2)(includes o187 p87)(includes o187 p142)(includes o187 p204)

(waiting o188)
(includes o188 p10)(includes o188 p38)(includes o188 p63)(includes o188 p72)(includes o188 p103)(includes o188 p141)(includes o188 p203)(includes o188 p216)(includes o188 p228)

(waiting o189)
(includes o189 p29)(includes o189 p38)(includes o189 p51)(includes o189 p55)(includes o189 p101)(includes o189 p202)

(waiting o190)
(includes o190 p28)(includes o190 p86)(includes o190 p193)(includes o190 p216)

(waiting o191)
(includes o191 p16)(includes o191 p64)(includes o191 p105)(includes o191 p111)(includes o191 p217)(includes o191 p223)

(waiting o192)
(includes o192 p26)(includes o192 p35)(includes o192 p67)(includes o192 p121)(includes o192 p127)(includes o192 p142)(includes o192 p165)(includes o192 p167)(includes o192 p202)

(waiting o193)
(includes o193 p22)(includes o193 p74)(includes o193 p91)(includes o193 p100)(includes o193 p137)(includes o193 p209)(includes o193 p228)

(waiting o194)
(includes o194 p3)(includes o194 p50)(includes o194 p203)(includes o194 p211)(includes o194 p216)

(waiting o195)
(includes o195 p30)(includes o195 p73)(includes o195 p79)(includes o195 p197)(includes o195 p227)

(waiting o196)
(includes o196 p28)(includes o196 p55)(includes o196 p115)(includes o196 p195)

(waiting o197)
(includes o197 p149)(includes o197 p170)(includes o197 p211)

(waiting o198)
(includes o198 p33)(includes o198 p69)(includes o198 p107)(includes o198 p128)(includes o198 p182)(includes o198 p218)

(waiting o199)
(includes o199 p16)(includes o199 p119)(includes o199 p151)(includes o199 p154)(includes o199 p164)(includes o199 p190)

(waiting o200)
(includes o200 p6)(includes o200 p97)(includes o200 p153)(includes o200 p209)(includes o200 p221)(includes o200 p224)

(waiting o201)
(includes o201 p30)(includes o201 p45)(includes o201 p60)(includes o201 p78)(includes o201 p82)(includes o201 p147)(includes o201 p156)(includes o201 p167)(includes o201 p196)

(waiting o202)
(includes o202 p99)(includes o202 p168)(includes o202 p221)

(waiting o203)
(includes o203 p19)(includes o203 p68)(includes o203 p121)(includes o203 p146)

(waiting o204)
(includes o204 p53)(includes o204 p72)(includes o204 p75)(includes o204 p105)(includes o204 p143)(includes o204 p185)

(waiting o205)
(includes o205 p13)(includes o205 p34)(includes o205 p41)(includes o205 p71)(includes o205 p108)(includes o205 p161)

(waiting o206)
(includes o206 p88)(includes o206 p136)(includes o206 p155)(includes o206 p184)(includes o206 p197)

(waiting o207)
(includes o207 p112)(includes o207 p169)(includes o207 p216)

(waiting o208)
(includes o208 p48)(includes o208 p107)(includes o208 p163)

(waiting o209)
(includes o209 p6)(includes o209 p18)(includes o209 p51)(includes o209 p96)(includes o209 p105)(includes o209 p109)(includes o209 p110)(includes o209 p145)(includes o209 p179)

(waiting o210)
(includes o210 p10)(includes o210 p117)(includes o210 p183)

(waiting o211)
(includes o211 p22)(includes o211 p56)(includes o211 p81)(includes o211 p101)(includes o211 p103)(includes o211 p126)(includes o211 p139)(includes o211 p154)(includes o211 p191)

(waiting o212)
(includes o212 p32)(includes o212 p73)(includes o212 p101)(includes o212 p186)(includes o212 p213)

(waiting o213)
(includes o213 p85)(includes o213 p100)(includes o213 p114)(includes o213 p164)(includes o213 p175)

(waiting o214)
(includes o214 p31)(includes o214 p33)(includes o214 p51)(includes o214 p73)(includes o214 p129)(includes o214 p160)(includes o214 p202)

(waiting o215)
(includes o215 p9)(includes o215 p18)(includes o215 p36)(includes o215 p60)(includes o215 p66)

(waiting o216)
(includes o216 p17)(includes o216 p35)(includes o216 p73)(includes o216 p106)(includes o216 p130)(includes o216 p131)(includes o216 p172)

(waiting o217)
(includes o217 p19)(includes o217 p122)(includes o217 p226)

(waiting o218)
(includes o218 p95)(includes o218 p101)(includes o218 p111)(includes o218 p148)(includes o218 p186)(includes o218 p209)(includes o218 p214)(includes o218 p215)(includes o218 p225)

(waiting o219)
(includes o219 p26)(includes o219 p42)(includes o219 p57)(includes o219 p116)(includes o219 p126)(includes o219 p143)(includes o219 p160)(includes o219 p214)(includes o219 p227)

(waiting o220)
(includes o220 p69)(includes o220 p158)(includes o220 p181)(includes o220 p215)(includes o220 p222)

(waiting o221)
(includes o221 p124)(includes o221 p148)(includes o221 p168)(includes o221 p179)(includes o221 p184)(includes o221 p206)

(waiting o222)
(includes o222 p42)(includes o222 p180)(includes o222 p217)(includes o222 p227)

(waiting o223)
(includes o223 p40)(includes o223 p78)(includes o223 p88)(includes o223 p131)

(waiting o224)
(includes o224 p24)(includes o224 p40)(includes o224 p41)(includes o224 p64)(includes o224 p164)

(waiting o225)
(includes o225 p39)(includes o225 p61)(includes o225 p93)(includes o225 p117)(includes o225 p137)(includes o225 p159)(includes o225 p208)

(waiting o226)
(includes o226 p95)(includes o226 p133)(includes o226 p227)

(waiting o227)
(includes o227 p30)(includes o227 p48)(includes o227 p63)(includes o227 p134)

(waiting o228)
(includes o228 p18)(includes o228 p58)(includes o228 p126)(includes o228 p138)(includes o228 p170)

(waiting o229)
(includes o229 p14)(includes o229 p32)(includes o229 p50)(includes o229 p59)(includes o229 p82)(includes o229 p119)(includes o229 p190)(includes o229 p204)

(waiting o230)
(includes o230 p33)(includes o230 p69)(includes o230 p71)(includes o230 p73)(includes o230 p87)(includes o230 p142)(includes o230 p179)

(not-made p1)
(not-made p2)
(not-made p3)
(not-made p4)
(not-made p5)
(not-made p6)
(not-made p7)
(not-made p8)
(not-made p9)
(not-made p10)
(not-made p11)
(not-made p12)
(not-made p13)
(not-made p14)
(not-made p15)
(not-made p16)
(not-made p17)
(not-made p18)
(not-made p19)
(not-made p20)
(not-made p21)
(not-made p22)
(not-made p23)
(not-made p24)
(not-made p25)
(not-made p26)
(not-made p27)
(not-made p28)
(not-made p29)
(not-made p30)
(not-made p31)
(not-made p32)
(not-made p33)
(not-made p34)
(not-made p35)
(not-made p36)
(not-made p37)
(not-made p38)
(not-made p39)
(not-made p40)
(not-made p41)
(not-made p42)
(not-made p43)
(not-made p44)
(not-made p45)
(not-made p46)
(not-made p47)
(not-made p48)
(not-made p49)
(not-made p50)
(not-made p51)
(not-made p52)
(not-made p53)
(not-made p54)
(not-made p55)
(not-made p56)
(not-made p57)
(not-made p58)
(not-made p59)
(not-made p60)
(not-made p61)
(not-made p62)
(not-made p63)
(not-made p64)
(not-made p65)
(not-made p66)
(not-made p67)
(not-made p68)
(not-made p69)
(not-made p70)
(not-made p71)
(not-made p72)
(not-made p73)
(not-made p74)
(not-made p75)
(not-made p76)
(not-made p77)
(not-made p78)
(not-made p79)
(not-made p80)
(not-made p81)
(not-made p82)
(not-made p83)
(not-made p84)
(not-made p85)
(not-made p86)
(not-made p87)
(not-made p88)
(not-made p89)
(not-made p90)
(not-made p91)
(not-made p92)
(not-made p93)
(not-made p94)
(not-made p95)
(not-made p96)
(not-made p97)
(not-made p98)
(not-made p99)
(not-made p100)
(not-made p101)
(not-made p102)
(not-made p103)
(not-made p104)
(not-made p105)
(not-made p106)
(not-made p107)
(not-made p108)
(not-made p109)
(not-made p110)
(not-made p111)
(not-made p112)
(not-made p113)
(not-made p114)
(not-made p115)
(not-made p116)
(not-made p117)
(not-made p118)
(not-made p119)
(not-made p120)
(not-made p121)
(not-made p122)
(not-made p123)
(not-made p124)
(not-made p125)
(not-made p126)
(not-made p127)
(not-made p128)
(not-made p129)
(not-made p130)
(not-made p131)
(not-made p132)
(not-made p133)
(not-made p134)
(not-made p135)
(not-made p136)
(not-made p137)
(not-made p138)
(not-made p139)
(not-made p140)
(not-made p141)
(not-made p142)
(not-made p143)
(not-made p144)
(not-made p145)
(not-made p146)
(not-made p147)
(not-made p148)
(not-made p149)
(not-made p150)
(not-made p151)
(not-made p152)
(not-made p153)
(not-made p154)
(not-made p155)
(not-made p156)
(not-made p157)
(not-made p158)
(not-made p159)
(not-made p160)
(not-made p161)
(not-made p162)
(not-made p163)
(not-made p164)
(not-made p165)
(not-made p166)
(not-made p167)
(not-made p168)
(not-made p169)
(not-made p170)
(not-made p171)
(not-made p172)
(not-made p173)
(not-made p174)
(not-made p175)
(not-made p176)
(not-made p177)
(not-made p178)
(not-made p179)
(not-made p180)
(not-made p181)
(not-made p182)
(not-made p183)
(not-made p184)
(not-made p185)
(not-made p186)
(not-made p187)
(not-made p188)
(not-made p189)
(not-made p190)
(not-made p191)
(not-made p192)
(not-made p193)
(not-made p194)
(not-made p195)
(not-made p196)
(not-made p197)
(not-made p198)
(not-made p199)
(not-made p200)
(not-made p201)
(not-made p202)
(not-made p203)
(not-made p204)
(not-made p205)
(not-made p206)
(not-made p207)
(not-made p208)
(not-made p209)
(not-made p210)
(not-made p211)
(not-made p212)
(not-made p213)
(not-made p214)
(not-made p215)
(not-made p216)
(not-made p217)
(not-made p218)
(not-made p219)
(not-made p220)
(not-made p221)
(not-made p222)
(not-made p223)
(not-made p224)
(not-made p225)
(not-made p226)
(not-made p227)
(not-made p228)
(not-made p229)
(not-made p230)

(= (total-cost) 0)

)

(:goal
(and
(shipped o1)
(shipped o2)
(shipped o3)
(shipped o4)
(shipped o5)
(shipped o6)
(shipped o7)
(shipped o8)
(shipped o9)
(shipped o10)
(shipped o11)
(shipped o12)
(shipped o13)
(shipped o14)
(shipped o15)
(shipped o16)
(shipped o17)
(shipped o18)
(shipped o19)
(shipped o20)
(shipped o21)
(shipped o22)
(shipped o23)
(shipped o24)
(shipped o25)
(shipped o26)
(shipped o27)
(shipped o28)
(shipped o29)
(shipped o30)
(shipped o31)
(shipped o32)
(shipped o33)
(shipped o34)
(shipped o35)
(shipped o36)
(shipped o37)
(shipped o38)
(shipped o39)
(shipped o40)
(shipped o41)
(shipped o42)
(shipped o43)
(shipped o44)
(shipped o45)
(shipped o46)
(shipped o47)
(shipped o48)
(shipped o49)
(shipped o50)
(shipped o51)
(shipped o52)
(shipped o53)
(shipped o54)
(shipped o55)
(shipped o56)
(shipped o57)
(shipped o58)
(shipped o59)
(shipped o60)
(shipped o61)
(shipped o62)
(shipped o63)
(shipped o64)
(shipped o65)
(shipped o66)
(shipped o67)
(shipped o68)
(shipped o69)
(shipped o70)
(shipped o71)
(shipped o72)
(shipped o73)
(shipped o74)
(shipped o75)
(shipped o76)
(shipped o77)
(shipped o78)
(shipped o79)
(shipped o80)
(shipped o81)
(shipped o82)
(shipped o83)
(shipped o84)
(shipped o85)
(shipped o86)
(shipped o87)
(shipped o88)
(shipped o89)
(shipped o90)
(shipped o91)
(shipped o92)
(shipped o93)
(shipped o94)
(shipped o95)
(shipped o96)
(shipped o97)
(shipped o98)
(shipped o99)
(shipped o100)
(shipped o101)
(shipped o102)
(shipped o103)
(shipped o104)
(shipped o105)
(shipped o106)
(shipped o107)
(shipped o108)
(shipped o109)
(shipped o110)
(shipped o111)
(shipped o112)
(shipped o113)
(shipped o114)
(shipped o115)
(shipped o116)
(shipped o117)
(shipped o118)
(shipped o119)
(shipped o120)
(shipped o121)
(shipped o122)
(shipped o123)
(shipped o124)
(shipped o125)
(shipped o126)
(shipped o127)
(shipped o128)
(shipped o129)
(shipped o130)
(shipped o131)
(shipped o132)
(shipped o133)
(shipped o134)
(shipped o135)
(shipped o136)
(shipped o137)
(shipped o138)
(shipped o139)
(shipped o140)
(shipped o141)
(shipped o142)
(shipped o143)
(shipped o144)
(shipped o145)
(shipped o146)
(shipped o147)
(shipped o148)
(shipped o149)
(shipped o150)
(shipped o151)
(shipped o152)
(shipped o153)
(shipped o154)
(shipped o155)
(shipped o156)
(shipped o157)
(shipped o158)
(shipped o159)
(shipped o160)
(shipped o161)
(shipped o162)
(shipped o163)
(shipped o164)
(shipped o165)
(shipped o166)
(shipped o167)
(shipped o168)
(shipped o169)
(shipped o170)
(shipped o171)
(shipped o172)
(shipped o173)
(shipped o174)
(shipped o175)
(shipped o176)
(shipped o177)
(shipped o178)
(shipped o179)
(shipped o180)
(shipped o181)
(shipped o182)
(shipped o183)
(shipped o184)
(shipped o185)
(shipped o186)
(shipped o187)
(shipped o188)
(shipped o189)
(shipped o190)
(shipped o191)
(shipped o192)
(shipped o193)
(shipped o194)
(shipped o195)
(shipped o196)
(shipped o197)
(shipped o198)
(shipped o199)
(shipped o200)
(shipped o201)
(shipped o202)
(shipped o203)
(shipped o204)
(shipped o205)
(shipped o206)
(shipped o207)
(shipped o208)
(shipped o209)
(shipped o210)
(shipped o211)
(shipped o212)
(shipped o213)
(shipped o214)
(shipped o215)
(shipped o216)
(shipped o217)
(shipped o218)
(shipped o219)
(shipped o220)
(shipped o221)
(shipped o222)
(shipped o223)
(shipped o224)
(shipped o225)
(shipped o226)
(shipped o227)
(shipped o228)
(shipped o229)
(shipped o230)
))

(:metric minimize (total-cost))

)

