(define (problem os-sequencedstrips-p230_2)
(:domain openstacks-sequencedstrips-nonADL-nonNegated)
(:objects 
n0 n1 n2 n3 n4 n5 n6 n7 n8 n9 n10 n11 n12 n13 n14 n15 n16 n17 n18 n19 n20 n21 n22 n23 n24 n25 n26 n27 n28 n29 n30 n31 n32 n33 n34 n35 n36 n37 n38 n39 n40 n41 n42 n43 n44 n45 n46 n47 n48 n49 n50 n51 n52 n53 n54 n55 n56 n57 n58 n59 n60 n61 n62 n63 n64 n65 n66 n67 n68 n69 n70 n71 n72 n73 n74 n75 n76 n77 n78 n79 n80 n81 n82 n83 n84 n85 n86 n87 n88 n89 n90 n91 n92 n93 n94 n95 n96 n97 n98 n99 n100 n101 n102 n103 n104 n105 n106 n107 n108 n109 n110 n111 n112 n113 n114 n115 n116 n117 n118 n119 n120 n121 n122 n123 n124 n125 n126 n127 n128 n129 n130 n131 n132 n133 n134 n135 n136 n137 n138 n139 n140 n141 n142 n143 n144 n145 n146 n147 n148 n149 n150 n151 n152 n153 n154 n155 n156 n157 n158 n159 n160 n161 n162 n163 n164 n165 n166 n167 n168 n169 n170 n171 n172 n173 n174 n175 n176 n177 n178 n179 n180 n181 n182 n183 n184 n185 n186 n187 n188 n189 n190 n191 n192 n193 n194 n195 n196 n197 n198 n199 n200 n201 n202 n203 n204 n205 n206 n207 n208 n209 n210 n211 n212 n213 n214 n215 n216 n217 n218 n219 n220 n221 n222 n223 n224 n225 n226 n227 n228 n229 n230  - count
)

(:init
(next-count n0 n1) (next-count n1 n2) (next-count n2 n3) (next-count n3 n4) (next-count n4 n5) (next-count n5 n6) (next-count n6 n7) (next-count n7 n8) (next-count n8 n9) (next-count n9 n10) (next-count n10 n11) (next-count n11 n12) (next-count n12 n13) (next-count n13 n14) (next-count n14 n15) (next-count n15 n16) (next-count n16 n17) (next-count n17 n18) (next-count n18 n19) (next-count n19 n20) (next-count n20 n21) (next-count n21 n22) (next-count n22 n23) (next-count n23 n24) (next-count n24 n25) (next-count n25 n26) (next-count n26 n27) (next-count n27 n28) (next-count n28 n29) (next-count n29 n30) (next-count n30 n31) (next-count n31 n32) (next-count n32 n33) (next-count n33 n34) (next-count n34 n35) (next-count n35 n36) (next-count n36 n37) (next-count n37 n38) (next-count n38 n39) (next-count n39 n40) (next-count n40 n41) (next-count n41 n42) (next-count n42 n43) (next-count n43 n44) (next-count n44 n45) (next-count n45 n46) (next-count n46 n47) (next-count n47 n48) (next-count n48 n49) (next-count n49 n50) (next-count n50 n51) (next-count n51 n52) (next-count n52 n53) (next-count n53 n54) (next-count n54 n55) (next-count n55 n56) (next-count n56 n57) (next-count n57 n58) (next-count n58 n59) (next-count n59 n60) (next-count n60 n61) (next-count n61 n62) (next-count n62 n63) (next-count n63 n64) (next-count n64 n65) (next-count n65 n66) (next-count n66 n67) (next-count n67 n68) (next-count n68 n69) (next-count n69 n70) (next-count n70 n71) (next-count n71 n72) (next-count n72 n73) (next-count n73 n74) (next-count n74 n75) (next-count n75 n76) (next-count n76 n77) (next-count n77 n78) (next-count n78 n79) (next-count n79 n80) (next-count n80 n81) (next-count n81 n82) (next-count n82 n83) (next-count n83 n84) (next-count n84 n85) (next-count n85 n86) (next-count n86 n87) (next-count n87 n88) (next-count n88 n89) (next-count n89 n90) (next-count n90 n91) (next-count n91 n92) (next-count n92 n93) (next-count n93 n94) (next-count n94 n95) (next-count n95 n96) (next-count n96 n97) (next-count n97 n98) (next-count n98 n99) (next-count n99 n100) (next-count n100 n101) (next-count n101 n102) (next-count n102 n103) (next-count n103 n104) (next-count n104 n105) (next-count n105 n106) (next-count n106 n107) (next-count n107 n108) (next-count n108 n109) (next-count n109 n110) (next-count n110 n111) (next-count n111 n112) (next-count n112 n113) (next-count n113 n114) (next-count n114 n115) (next-count n115 n116) (next-count n116 n117) (next-count n117 n118) (next-count n118 n119) (next-count n119 n120) (next-count n120 n121) (next-count n121 n122) (next-count n122 n123) (next-count n123 n124) (next-count n124 n125) (next-count n125 n126) (next-count n126 n127) (next-count n127 n128) (next-count n128 n129) (next-count n129 n130) (next-count n130 n131) (next-count n131 n132) (next-count n132 n133) (next-count n133 n134) (next-count n134 n135) (next-count n135 n136) (next-count n136 n137) (next-count n137 n138) (next-count n138 n139) (next-count n139 n140) (next-count n140 n141) (next-count n141 n142) (next-count n142 n143) (next-count n143 n144) (next-count n144 n145) (next-count n145 n146) (next-count n146 n147) (next-count n147 n148) (next-count n148 n149) (next-count n149 n150) (next-count n150 n151) (next-count n151 n152) (next-count n152 n153) (next-count n153 n154) (next-count n154 n155) (next-count n155 n156) (next-count n156 n157) (next-count n157 n158) (next-count n158 n159) (next-count n159 n160) (next-count n160 n161) (next-count n161 n162) (next-count n162 n163) (next-count n163 n164) (next-count n164 n165) (next-count n165 n166) (next-count n166 n167) (next-count n167 n168) (next-count n168 n169) (next-count n169 n170) (next-count n170 n171) (next-count n171 n172) (next-count n172 n173) (next-count n173 n174) (next-count n174 n175) (next-count n175 n176) (next-count n176 n177) (next-count n177 n178) (next-count n178 n179) (next-count n179 n180) (next-count n180 n181) (next-count n181 n182) (next-count n182 n183) (next-count n183 n184) (next-count n184 n185) (next-count n185 n186) (next-count n186 n187) (next-count n187 n188) (next-count n188 n189) (next-count n189 n190) (next-count n190 n191) (next-count n191 n192) (next-count n192 n193) (next-count n193 n194) (next-count n194 n195) (next-count n195 n196) (next-count n196 n197) (next-count n197 n198) (next-count n198 n199) (next-count n199 n200) (next-count n200 n201) (next-count n201 n202) (next-count n202 n203) (next-count n203 n204) (next-count n204 n205) (next-count n205 n206) (next-count n206 n207) (next-count n207 n208) (next-count n208 n209) (next-count n209 n210) (next-count n210 n211) (next-count n211 n212) (next-count n212 n213) (next-count n213 n214) (next-count n214 n215) (next-count n215 n216) (next-count n216 n217) (next-count n217 n218) (next-count n218 n219) (next-count n219 n220) (next-count n220 n221) (next-count n221 n222) (next-count n222 n223) (next-count n223 n224) (next-count n224 n225) (next-count n225 n226) (next-count n226 n227) (next-count n227 n228) (next-count n228 n229) (next-count n229 n230) 
(stacks-avail n0)

(waiting o1)
(includes o1 p12)(includes o1 p126)(includes o1 p149)(includes o1 p179)(includes o1 p206)(includes o1 p212)

(waiting o2)
(includes o2 p41)(includes o2 p101)(includes o2 p134)(includes o2 p210)(includes o2 p226)

(waiting o3)
(includes o3 p41)(includes o3 p80)(includes o3 p85)(includes o3 p134)(includes o3 p166)(includes o3 p197)(includes o3 p215)

(waiting o4)
(includes o4 p34)(includes o4 p65)(includes o4 p71)(includes o4 p117)(includes o4 p133)(includes o4 p141)(includes o4 p151)(includes o4 p156)(includes o4 p204)

(waiting o5)
(includes o5 p158)(includes o5 p200)(includes o5 p215)

(waiting o6)
(includes o6 p1)(includes o6 p28)(includes o6 p48)(includes o6 p65)(includes o6 p70)(includes o6 p85)(includes o6 p129)(includes o6 p206)(includes o6 p209)(includes o6 p212)

(waiting o7)
(includes o7 p8)(includes o7 p83)(includes o7 p203)

(waiting o8)
(includes o8 p117)(includes o8 p118)(includes o8 p145)

(waiting o9)
(includes o9 p37)(includes o9 p50)(includes o9 p59)(includes o9 p101)(includes o9 p104)(includes o9 p116)(includes o9 p154)(includes o9 p181)(includes o9 p207)

(waiting o10)
(includes o10 p86)(includes o10 p87)(includes o10 p131)(includes o10 p160)

(waiting o11)
(includes o11 p5)(includes o11 p21)(includes o11 p61)(includes o11 p119)(includes o11 p123)(includes o11 p139)

(waiting o12)
(includes o12 p45)(includes o12 p80)(includes o12 p113)(includes o12 p120)(includes o12 p160)(includes o12 p166)

(waiting o13)
(includes o13 p11)(includes o13 p33)(includes o13 p45)(includes o13 p54)(includes o13 p217)(includes o13 p218)

(waiting o14)
(includes o14 p4)(includes o14 p11)(includes o14 p94)(includes o14 p95)(includes o14 p181)(includes o14 p211)(includes o14 p214)

(waiting o15)
(includes o15 p66)(includes o15 p77)(includes o15 p102)(includes o15 p105)(includes o15 p114)(includes o15 p147)(includes o15 p177)(includes o15 p210)

(waiting o16)
(includes o16 p35)(includes o16 p114)(includes o16 p198)(includes o16 p210)(includes o16 p223)

(waiting o17)
(includes o17 p46)(includes o17 p48)(includes o17 p50)(includes o17 p70)(includes o17 p82)(includes o17 p87)(includes o17 p95)(includes o17 p114)(includes o17 p160)(includes o17 p194)(includes o17 p200)

(waiting o18)
(includes o18 p9)(includes o18 p55)(includes o18 p95)(includes o18 p141)(includes o18 p161)(includes o18 p173)

(waiting o19)
(includes o19 p7)(includes o19 p89)(includes o19 p101)(includes o19 p124)(includes o19 p131)(includes o19 p230)

(waiting o20)
(includes o20 p62)(includes o20 p218)

(waiting o21)
(includes o21 p134)(includes o21 p137)(includes o21 p147)(includes o21 p151)(includes o21 p183)(includes o21 p200)(includes o21 p218)

(waiting o22)
(includes o22 p20)(includes o22 p21)(includes o22 p28)(includes o22 p59)(includes o22 p68)(includes o22 p135)(includes o22 p138)(includes o22 p187)(includes o22 p221)(includes o22 p229)

(waiting o23)
(includes o23 p28)(includes o23 p36)(includes o23 p43)(includes o23 p60)(includes o23 p67)(includes o23 p85)(includes o23 p182)

(waiting o24)
(includes o24 p33)(includes o24 p90)(includes o24 p158)(includes o24 p162)(includes o24 p205)(includes o24 p214)

(waiting o25)
(includes o25 p11)(includes o25 p54)(includes o25 p95)(includes o25 p98)(includes o25 p156)(includes o25 p223)(includes o25 p230)

(waiting o26)
(includes o26 p86)(includes o26 p122)(includes o26 p139)(includes o26 p147)(includes o26 p210)

(waiting o27)
(includes o27 p86)(includes o27 p149)(includes o27 p159)(includes o27 p172)(includes o27 p188)(includes o27 p219)

(waiting o28)
(includes o28 p27)(includes o28 p86)(includes o28 p138)

(waiting o29)
(includes o29 p21)(includes o29 p36)(includes o29 p73)(includes o29 p120)(includes o29 p137)(includes o29 p157)(includes o29 p196)(includes o29 p197)

(waiting o30)
(includes o30 p5)(includes o30 p28)(includes o30 p31)(includes o30 p44)(includes o30 p46)(includes o30 p47)(includes o30 p74)(includes o30 p131)(includes o30 p188)

(waiting o31)
(includes o31 p5)(includes o31 p33)(includes o31 p79)(includes o31 p105)(includes o31 p109)(includes o31 p119)

(waiting o32)
(includes o32 p2)(includes o32 p60)(includes o32 p162)(includes o32 p203)

(waiting o33)
(includes o33 p31)(includes o33 p33)(includes o33 p67)(includes o33 p87)(includes o33 p163)(includes o33 p206)

(waiting o34)
(includes o34 p19)(includes o34 p25)(includes o34 p31)(includes o34 p40)(includes o34 p70)(includes o34 p119)(includes o34 p172)(includes o34 p179)(includes o34 p185)(includes o34 p187)(includes o34 p216)(includes o34 p223)

(waiting o35)
(includes o35 p64)(includes o35 p65)(includes o35 p67)(includes o35 p100)(includes o35 p165)

(waiting o36)
(includes o36 p33)(includes o36 p119)(includes o36 p155)

(waiting o37)
(includes o37 p15)(includes o37 p27)(includes o37 p59)(includes o37 p76)(includes o37 p87)(includes o37 p131)(includes o37 p182)

(waiting o38)
(includes o38 p20)(includes o38 p41)(includes o38 p74)(includes o38 p90)(includes o38 p100)(includes o38 p127)(includes o38 p173)(includes o38 p199)(includes o38 p217)

(waiting o39)
(includes o39 p77)(includes o39 p118)(includes o39 p131)(includes o39 p222)

(waiting o40)
(includes o40 p71)

(waiting o41)
(includes o41 p19)(includes o41 p68)(includes o41 p78)(includes o41 p164)(includes o41 p209)

(waiting o42)
(includes o42 p80)(includes o42 p83)(includes o42 p99)(includes o42 p107)(includes o42 p117)(includes o42 p171)(includes o42 p184)

(waiting o43)
(includes o43 p69)(includes o43 p74)

(waiting o44)
(includes o44 p49)(includes o44 p55)(includes o44 p56)(includes o44 p104)(includes o44 p173)(includes o44 p181)

(waiting o45)
(includes o45 p50)(includes o45 p81)(includes o45 p136)(includes o45 p210)

(waiting o46)
(includes o46 p26)(includes o46 p145)(includes o46 p182)(includes o46 p183)

(waiting o47)
(includes o47 p22)(includes o47 p31)(includes o47 p86)(includes o47 p173)

(waiting o48)
(includes o48 p75)(includes o48 p79)(includes o48 p91)(includes o48 p116)(includes o48 p146)(includes o48 p153)(includes o48 p224)

(waiting o49)
(includes o49 p30)(includes o49 p34)(includes o49 p93)(includes o49 p109)(includes o49 p135)(includes o49 p204)(includes o49 p213)

(waiting o50)
(includes o50 p7)(includes o50 p129)(includes o50 p174)(includes o50 p204)(includes o50 p208)

(waiting o51)
(includes o51 p8)(includes o51 p9)(includes o51 p34)(includes o51 p65)(includes o51 p67)(includes o51 p164)(includes o51 p217)(includes o51 p218)

(waiting o52)
(includes o52 p17)(includes o52 p21)(includes o52 p44)(includes o52 p61)(includes o52 p70)(includes o52 p158)(includes o52 p159)(includes o52 p168)(includes o52 p172)(includes o52 p208)

(waiting o53)
(includes o53 p16)(includes o53 p21)(includes o53 p87)(includes o53 p158)(includes o53 p163)(includes o53 p227)

(waiting o54)
(includes o54 p29)(includes o54 p32)(includes o54 p67)(includes o54 p137)(includes o54 p153)(includes o54 p186)(includes o54 p188)(includes o54 p195)(includes o54 p204)

(waiting o55)
(includes o55 p65)(includes o55 p206)(includes o55 p227)

(waiting o56)
(includes o56 p26)(includes o56 p37)(includes o56 p58)(includes o56 p135)(includes o56 p147)(includes o56 p148)(includes o56 p174)(includes o56 p189)(includes o56 p217)

(waiting o57)
(includes o57 p39)(includes o57 p44)(includes o57 p66)(includes o57 p127)(includes o57 p192)(includes o57 p200)(includes o57 p201)(includes o57 p210)

(waiting o58)
(includes o58 p29)(includes o58 p51)(includes o58 p106)(includes o58 p139)(includes o58 p181)(includes o58 p189)(includes o58 p210)(includes o58 p230)

(waiting o59)
(includes o59 p48)(includes o59 p178)(includes o59 p226)

(waiting o60)
(includes o60 p58)(includes o60 p123)(includes o60 p132)(includes o60 p141)(includes o60 p151)

(waiting o61)
(includes o61 p73)(includes o61 p137)(includes o61 p176)(includes o61 p224)

(waiting o62)
(includes o62 p11)(includes o62 p50)(includes o62 p65)(includes o62 p72)(includes o62 p123)(includes o62 p126)(includes o62 p163)(includes o62 p165)

(waiting o63)
(includes o63 p9)(includes o63 p41)(includes o63 p64)(includes o63 p103)(includes o63 p105)

(waiting o64)
(includes o64 p2)(includes o64 p52)(includes o64 p65)(includes o64 p96)(includes o64 p104)(includes o64 p143)

(waiting o65)
(includes o65 p8)(includes o65 p49)(includes o65 p89)(includes o65 p122)(includes o65 p123)

(waiting o66)
(includes o66 p38)(includes o66 p44)(includes o66 p49)(includes o66 p114)(includes o66 p189)(includes o66 p198)(includes o66 p200)

(waiting o67)
(includes o67 p6)(includes o67 p25)(includes o67 p42)(includes o67 p74)(includes o67 p124)(includes o67 p162)(includes o67 p164)(includes o67 p230)

(waiting o68)
(includes o68 p34)(includes o68 p98)(includes o68 p129)(includes o68 p169)(includes o68 p172)(includes o68 p177)(includes o68 p187)(includes o68 p223)(includes o68 p227)

(waiting o69)
(includes o69 p61)(includes o69 p146)

(waiting o70)
(includes o70 p1)(includes o70 p12)(includes o70 p14)(includes o70 p30)(includes o70 p40)(includes o70 p63)(includes o70 p100)(includes o70 p133)

(waiting o71)
(includes o71 p136)(includes o71 p215)

(waiting o72)
(includes o72 p47)(includes o72 p55)(includes o72 p96)

(waiting o73)
(includes o73 p153)(includes o73 p167)(includes o73 p195)(includes o73 p208)

(waiting o74)
(includes o74 p6)(includes o74 p16)(includes o74 p74)(includes o74 p97)(includes o74 p135)

(waiting o75)
(includes o75 p27)(includes o75 p92)(includes o75 p104)(includes o75 p118)(includes o75 p128)(includes o75 p145)(includes o75 p149)(includes o75 p172)(includes o75 p179)(includes o75 p183)

(waiting o76)
(includes o76 p62)(includes o76 p88)(includes o76 p195)(includes o76 p196)(includes o76 p202)(includes o76 p228)

(waiting o77)
(includes o77 p37)(includes o77 p110)(includes o77 p152)(includes o77 p166)(includes o77 p229)

(waiting o78)
(includes o78 p11)(includes o78 p49)(includes o78 p53)(includes o78 p63)(includes o78 p72)(includes o78 p84)(includes o78 p115)(includes o78 p219)(includes o78 p228)

(waiting o79)
(includes o79 p15)(includes o79 p21)(includes o79 p40)(includes o79 p125)(includes o79 p138)(includes o79 p141)(includes o79 p213)(includes o79 p215)(includes o79 p224)

(waiting o80)
(includes o80 p8)(includes o80 p19)(includes o80 p49)(includes o80 p176)(includes o80 p199)

(waiting o81)
(includes o81 p168)(includes o81 p205)(includes o81 p219)

(waiting o82)
(includes o82 p73)(includes o82 p89)(includes o82 p120)(includes o82 p157)(includes o82 p159)(includes o82 p166)(includes o82 p186)

(waiting o83)
(includes o83 p30)(includes o83 p31)(includes o83 p40)(includes o83 p48)(includes o83 p101)(includes o83 p119)(includes o83 p169)(includes o83 p213)

(waiting o84)
(includes o84 p12)(includes o84 p51)(includes o84 p60)(includes o84 p141)(includes o84 p187)

(waiting o85)
(includes o85 p6)(includes o85 p52)(includes o85 p59)(includes o85 p60)(includes o85 p71)(includes o85 p93)(includes o85 p229)

(waiting o86)
(includes o86 p22)(includes o86 p32)(includes o86 p60)(includes o86 p98)(includes o86 p104)(includes o86 p127)(includes o86 p154)(includes o86 p160)(includes o86 p228)

(waiting o87)
(includes o87 p24)(includes o87 p61)(includes o87 p111)(includes o87 p144)

(waiting o88)
(includes o88 p82)(includes o88 p97)(includes o88 p126)(includes o88 p191)(includes o88 p192)(includes o88 p220)

(waiting o89)
(includes o89 p2)(includes o89 p60)(includes o89 p80)(includes o89 p105)(includes o89 p122)(includes o89 p140)(includes o89 p193)

(waiting o90)
(includes o90 p43)(includes o90 p75)(includes o90 p124)(includes o90 p143)(includes o90 p147)(includes o90 p168)(includes o90 p179)(includes o90 p182)(includes o90 p208)

(waiting o91)
(includes o91 p36)(includes o91 p67)(includes o91 p104)(includes o91 p117)(includes o91 p202)

(waiting o92)
(includes o92 p75)(includes o92 p78)(includes o92 p79)(includes o92 p112)(includes o92 p147)(includes o92 p153)(includes o92 p197)(includes o92 p215)(includes o92 p228)

(waiting o93)
(includes o93 p25)(includes o93 p60)(includes o93 p62)(includes o93 p85)(includes o93 p108)(includes o93 p142)(includes o93 p173)

(waiting o94)
(includes o94 p21)(includes o94 p23)(includes o94 p88)(includes o94 p114)(includes o94 p150)(includes o94 p170)

(waiting o95)
(includes o95 p36)(includes o95 p74)(includes o95 p182)(includes o95 p185)(includes o95 p192)(includes o95 p227)

(waiting o96)
(includes o96 p73)(includes o96 p99)(includes o96 p171)(includes o96 p224)

(waiting o97)
(includes o97 p20)(includes o97 p55)(includes o97 p62)(includes o97 p70)(includes o97 p87)(includes o97 p93)(includes o97 p170)(includes o97 p172)(includes o97 p184)(includes o97 p195)(includes o97 p222)

(waiting o98)
(includes o98 p29)(includes o98 p62)(includes o98 p94)(includes o98 p135)(includes o98 p140)(includes o98 p146)

(waiting o99)
(includes o99 p168)(includes o99 p184)(includes o99 p213)

(waiting o100)
(includes o100 p72)(includes o100 p77)(includes o100 p108)(includes o100 p126)(includes o100 p222)

(waiting o101)
(includes o101 p57)(includes o101 p88)(includes o101 p118)(includes o101 p208)(includes o101 p209)

(waiting o102)
(includes o102 p24)(includes o102 p29)(includes o102 p45)(includes o102 p78)(includes o102 p105)(includes o102 p120)(includes o102 p146)(includes o102 p167)(includes o102 p171)(includes o102 p174)(includes o102 p209)(includes o102 p222)(includes o102 p228)(includes o102 p230)

(waiting o103)
(includes o103 p79)(includes o103 p97)(includes o103 p123)(includes o103 p129)

(waiting o104)
(includes o104 p39)(includes o104 p107)(includes o104 p186)(includes o104 p194)(includes o104 p211)

(waiting o105)
(includes o105 p110)(includes o105 p210)

(waiting o106)
(includes o106 p35)(includes o106 p54)(includes o106 p75)(includes o106 p139)(includes o106 p152)(includes o106 p169)

(waiting o107)
(includes o107 p20)(includes o107 p22)(includes o107 p57)(includes o107 p66)(includes o107 p109)(includes o107 p123)

(waiting o108)
(includes o108 p57)(includes o108 p85)(includes o108 p140)(includes o108 p156)(includes o108 p208)(includes o108 p209)

(waiting o109)
(includes o109 p53)(includes o109 p104)(includes o109 p135)(includes o109 p227)

(waiting o110)
(includes o110 p13)(includes o110 p78)(includes o110 p88)(includes o110 p112)(includes o110 p125)(includes o110 p216)

(waiting o111)
(includes o111 p4)(includes o111 p21)(includes o111 p43)(includes o111 p85)(includes o111 p87)(includes o111 p177)

(waiting o112)
(includes o112 p107)(includes o112 p157)(includes o112 p164)

(waiting o113)
(includes o113 p10)(includes o113 p32)(includes o113 p78)(includes o113 p100)(includes o113 p137)

(waiting o114)
(includes o114 p213)

(waiting o115)
(includes o115 p56)(includes o115 p87)(includes o115 p114)(includes o115 p138)(includes o115 p176)

(waiting o116)
(includes o116 p27)(includes o116 p34)(includes o116 p37)(includes o116 p124)(includes o116 p134)(includes o116 p161)(includes o116 p172)(includes o116 p174)(includes o116 p191)(includes o116 p194)(includes o116 p196)(includes o116 p211)(includes o116 p220)(includes o116 p223)

(waiting o117)
(includes o117 p9)(includes o117 p26)(includes o117 p142)(includes o117 p152)(includes o117 p204)

(waiting o118)
(includes o118 p82)(includes o118 p128)(includes o118 p165)(includes o118 p180)(includes o118 p182)

(waiting o119)
(includes o119 p43)(includes o119 p51)(includes o119 p64)(includes o119 p72)(includes o119 p190)(includes o119 p211)(includes o119 p222)

(waiting o120)
(includes o120 p161)(includes o120 p178)(includes o120 p211)(includes o120 p215)(includes o120 p221)(includes o120 p222)

(waiting o121)
(includes o121 p43)(includes o121 p80)(includes o121 p113)(includes o121 p124)

(waiting o122)
(includes o122 p37)(includes o122 p101)(includes o122 p120)(includes o122 p160)(includes o122 p190)(includes o122 p202)

(waiting o123)
(includes o123 p107)(includes o123 p138)(includes o123 p187)(includes o123 p229)

(waiting o124)
(includes o124 p27)(includes o124 p28)(includes o124 p36)(includes o124 p40)(includes o124 p65)(includes o124 p106)(includes o124 p148)(includes o124 p184)(includes o124 p206)

(waiting o125)
(includes o125 p78)(includes o125 p125)(includes o125 p134)(includes o125 p156)

(waiting o126)
(includes o126 p2)(includes o126 p35)(includes o126 p108)(includes o126 p138)(includes o126 p189)(includes o126 p215)(includes o126 p218)(includes o126 p230)

(waiting o127)
(includes o127 p63)(includes o127 p83)(includes o127 p87)(includes o127 p109)(includes o127 p210)(includes o127 p215)

(waiting o128)
(includes o128 p24)(includes o128 p39)(includes o128 p98)(includes o128 p152)(includes o128 p189)(includes o128 p193)(includes o128 p217)

(waiting o129)
(includes o129 p61)(includes o129 p118)(includes o129 p124)(includes o129 p131)(includes o129 p157)(includes o129 p197)

(waiting o130)
(includes o130 p90)(includes o130 p121)(includes o130 p199)(includes o130 p208)(includes o130 p220)

(waiting o131)
(includes o131 p21)(includes o131 p43)(includes o131 p98)(includes o131 p107)(includes o131 p193)(includes o131 p207)(includes o131 p216)(includes o131 p225)(includes o131 p226)

(waiting o132)
(includes o132 p4)(includes o132 p74)(includes o132 p199)(includes o132 p208)(includes o132 p211)

(waiting o133)
(includes o133 p143)(includes o133 p156)(includes o133 p199)(includes o133 p224)

(waiting o134)
(includes o134 p10)(includes o134 p37)(includes o134 p44)(includes o134 p46)(includes o134 p79)(includes o134 p112)(includes o134 p131)(includes o134 p161)(includes o134 p210)

(waiting o135)
(includes o135 p94)(includes o135 p144)(includes o135 p147)(includes o135 p194)(includes o135 p217)(includes o135 p218)

(waiting o136)
(includes o136 p7)(includes o136 p148)(includes o136 p205)

(waiting o137)
(includes o137 p6)(includes o137 p37)(includes o137 p39)(includes o137 p72)(includes o137 p73)(includes o137 p197)

(waiting o138)
(includes o138 p5)(includes o138 p216)

(waiting o139)
(includes o139 p48)(includes o139 p163)(includes o139 p168)(includes o139 p206)

(waiting o140)
(includes o140 p35)(includes o140 p41)(includes o140 p56)(includes o140 p79)(includes o140 p108)(includes o140 p134)(includes o140 p142)(includes o140 p194)(includes o140 p228)

(waiting o141)
(includes o141 p35)(includes o141 p130)(includes o141 p187)(includes o141 p189)(includes o141 p221)

(waiting o142)
(includes o142 p24)(includes o142 p40)(includes o142 p100)(includes o142 p162)(includes o142 p172)

(waiting o143)
(includes o143 p1)(includes o143 p64)(includes o143 p83)(includes o143 p179)

(waiting o144)
(includes o144 p16)(includes o144 p20)(includes o144 p38)(includes o144 p60)(includes o144 p98)(includes o144 p151)(includes o144 p185)

(waiting o145)
(includes o145 p13)(includes o145 p26)(includes o145 p84)

(waiting o146)
(includes o146 p26)(includes o146 p50)(includes o146 p69)(includes o146 p88)(includes o146 p127)(includes o146 p204)

(waiting o147)
(includes o147 p42)(includes o147 p47)(includes o147 p97)(includes o147 p129)(includes o147 p143)(includes o147 p184)(includes o147 p185)

(waiting o148)
(includes o148 p2)(includes o148 p10)(includes o148 p15)(includes o148 p23)(includes o148 p36)(includes o148 p66)(includes o148 p92)(includes o148 p93)(includes o148 p121)(includes o148 p148)(includes o148 p169)(includes o148 p175)(includes o148 p199)

(waiting o149)
(includes o149 p1)(includes o149 p19)(includes o149 p21)(includes o149 p30)(includes o149 p52)(includes o149 p58)(includes o149 p70)(includes o149 p125)(includes o149 p137)(includes o149 p148)

(waiting o150)
(includes o150 p2)(includes o150 p10)(includes o150 p15)(includes o150 p32)(includes o150 p72)(includes o150 p176)(includes o150 p222)(includes o150 p226)

(waiting o151)
(includes o151 p54)(includes o151 p90)(includes o151 p160)(includes o151 p187)(includes o151 p191)(includes o151 p193)(includes o151 p218)(includes o151 p223)

(waiting o152)
(includes o152 p61)(includes o152 p78)(includes o152 p86)(includes o152 p149)(includes o152 p206)

(waiting o153)
(includes o153 p49)(includes o153 p82)(includes o153 p148)(includes o153 p211)

(waiting o154)
(includes o154 p14)(includes o154 p73)(includes o154 p93)(includes o154 p121)(includes o154 p144)(includes o154 p149)(includes o154 p184)(includes o154 p191)(includes o154 p214)

(waiting o155)
(includes o155 p20)(includes o155 p44)(includes o155 p75)(includes o155 p123)(includes o155 p194)(includes o155 p218)

(waiting o156)
(includes o156 p134)(includes o156 p179)(includes o156 p190)(includes o156 p205)(includes o156 p211)(includes o156 p212)

(waiting o157)
(includes o157 p46)(includes o157 p54)(includes o157 p58)(includes o157 p115)(includes o157 p121)(includes o157 p173)(includes o157 p179)

(waiting o158)
(includes o158 p13)(includes o158 p191)

(waiting o159)
(includes o159 p92)(includes o159 p104)(includes o159 p118)(includes o159 p162)

(waiting o160)
(includes o160 p76)(includes o160 p93)(includes o160 p127)(includes o160 p135)(includes o160 p205)

(waiting o161)
(includes o161 p62)(includes o161 p115)(includes o161 p119)(includes o161 p157)(includes o161 p191)(includes o161 p192)(includes o161 p224)(includes o161 p228)

(waiting o162)
(includes o162 p114)(includes o162 p140)(includes o162 p149)(includes o162 p150)(includes o162 p161)(includes o162 p210)(includes o162 p225)

(waiting o163)
(includes o163 p156)(includes o163 p222)(includes o163 p223)

(waiting o164)
(includes o164 p18)(includes o164 p41)(includes o164 p73)(includes o164 p134)(includes o164 p201)(includes o164 p218)

(waiting o165)
(includes o165 p55)(includes o165 p114)(includes o165 p167)

(waiting o166)
(includes o166 p27)(includes o166 p50)(includes o166 p100)(includes o166 p128)(includes o166 p151)(includes o166 p156)(includes o166 p182)

(waiting o167)
(includes o167 p22)(includes o167 p89)(includes o167 p106)(includes o167 p113)(includes o167 p185)(includes o167 p191)

(waiting o168)
(includes o168 p24)(includes o168 p31)(includes o168 p85)(includes o168 p90)(includes o168 p132)(includes o168 p158)

(waiting o169)
(includes o169 p30)(includes o169 p32)(includes o169 p58)(includes o169 p87)(includes o169 p115)(includes o169 p154)(includes o169 p176)(includes o169 p187)(includes o169 p211)

(waiting o170)
(includes o170 p73)(includes o170 p95)(includes o170 p106)(includes o170 p110)(includes o170 p152)(includes o170 p154)(includes o170 p184)(includes o170 p194)

(waiting o171)
(includes o171 p38)(includes o171 p98)(includes o171 p142)(includes o171 p201)

(waiting o172)
(includes o172 p30)(includes o172 p102)(includes o172 p124)(includes o172 p146)(includes o172 p228)

(waiting o173)
(includes o173 p23)(includes o173 p36)(includes o173 p61)(includes o173 p83)(includes o173 p84)(includes o173 p103)(includes o173 p117)(includes o173 p124)(includes o173 p166)(includes o173 p195)

(waiting o174)
(includes o174 p53)(includes o174 p100)(includes o174 p104)(includes o174 p123)

(waiting o175)
(includes o175 p48)(includes o175 p61)(includes o175 p86)(includes o175 p90)(includes o175 p228)

(waiting o176)
(includes o176 p87)(includes o176 p88)(includes o176 p162)(includes o176 p189)(includes o176 p226)

(waiting o177)
(includes o177 p18)(includes o177 p52)(includes o177 p81)(includes o177 p107)(includes o177 p120)(includes o177 p144)(includes o177 p160)(includes o177 p184)

(waiting o178)
(includes o178 p31)(includes o178 p61)(includes o178 p86)(includes o178 p115)(includes o178 p124)(includes o178 p133)(includes o178 p136)(includes o178 p154)(includes o178 p181)

(waiting o179)
(includes o179 p14)(includes o179 p116)(includes o179 p132)(includes o179 p138)(includes o179 p167)(includes o179 p174)(includes o179 p183)(includes o179 p201)(includes o179 p204)

(waiting o180)
(includes o180 p12)(includes o180 p54)(includes o180 p101)(includes o180 p120)(includes o180 p162)(includes o180 p202)(includes o180 p206)

(waiting o181)
(includes o181 p21)(includes o181 p94)(includes o181 p111)(includes o181 p178)

(waiting o182)
(includes o182 p21)(includes o182 p130)

(waiting o183)
(includes o183 p16)(includes o183 p49)(includes o183 p71)(includes o183 p85)(includes o183 p142)(includes o183 p164)(includes o183 p180)(includes o183 p186)(includes o183 p216)

(waiting o184)
(includes o184 p23)(includes o184 p31)(includes o184 p68)(includes o184 p105)(includes o184 p107)(includes o184 p119)(includes o184 p125)(includes o184 p204)(includes o184 p222)

(waiting o185)
(includes o185 p16)(includes o185 p36)(includes o185 p55)

(waiting o186)
(includes o186 p36)(includes o186 p90)(includes o186 p113)(includes o186 p127)(includes o186 p136)

(waiting o187)
(includes o187 p35)(includes o187 p41)(includes o187 p157)(includes o187 p161)(includes o187 p216)

(waiting o188)
(includes o188 p55)(includes o188 p133)(includes o188 p206)

(waiting o189)
(includes o189 p57)(includes o189 p71)(includes o189 p80)(includes o189 p138)(includes o189 p144)(includes o189 p151)(includes o189 p200)

(waiting o190)
(includes o190 p41)(includes o190 p54)(includes o190 p143)(includes o190 p166)(includes o190 p207)(includes o190 p218)

(waiting o191)
(includes o191 p40)(includes o191 p65)(includes o191 p67)(includes o191 p140)(includes o191 p150)(includes o191 p160)

(waiting o192)
(includes o192 p11)(includes o192 p16)(includes o192 p63)(includes o192 p164)(includes o192 p199)

(waiting o193)
(includes o193 p89)(includes o193 p98)(includes o193 p111)(includes o193 p143)(includes o193 p181)

(waiting o194)
(includes o194 p49)(includes o194 p59)(includes o194 p136)(includes o194 p163)(includes o194 p190)(includes o194 p196)(includes o194 p199)(includes o194 p211)

(waiting o195)
(includes o195 p4)(includes o195 p5)(includes o195 p84)(includes o195 p86)(includes o195 p87)(includes o195 p96)(includes o195 p128)(includes o195 p163)

(waiting o196)
(includes o196 p11)(includes o196 p69)(includes o196 p100)(includes o196 p205)(includes o196 p214)

(waiting o197)
(includes o197 p47)(includes o197 p94)(includes o197 p105)(includes o197 p183)

(waiting o198)
(includes o198 p17)(includes o198 p83)(includes o198 p87)(includes o198 p93)(includes o198 p118)(includes o198 p121)(includes o198 p194)(includes o198 p198)

(waiting o199)
(includes o199 p16)(includes o199 p53)(includes o199 p122)(includes o199 p133)(includes o199 p191)

(waiting o200)
(includes o200 p136)(includes o200 p144)(includes o200 p217)(includes o200 p222)

(waiting o201)
(includes o201 p31)(includes o201 p62)(includes o201 p102)(includes o201 p125)(includes o201 p163)(includes o201 p220)

(waiting o202)
(includes o202 p41)(includes o202 p91)(includes o202 p92)(includes o202 p152)(includes o202 p207)(includes o202 p213)(includes o202 p230)

(waiting o203)
(includes o203 p31)(includes o203 p44)(includes o203 p50)(includes o203 p132)(includes o203 p209)

(waiting o204)
(includes o204 p3)(includes o204 p32)(includes o204 p44)(includes o204 p54)(includes o204 p70)(includes o204 p114)(includes o204 p148)(includes o204 p179)(includes o204 p198)

(waiting o205)
(includes o205 p90)(includes o205 p181)(includes o205 p184)(includes o205 p187)

(waiting o206)
(includes o206 p9)(includes o206 p34)(includes o206 p54)(includes o206 p105)(includes o206 p132)(includes o206 p209)

(waiting o207)
(includes o207 p19)(includes o207 p22)(includes o207 p41)(includes o207 p47)(includes o207 p49)(includes o207 p77)(includes o207 p156)(includes o207 p173)(includes o207 p185)(includes o207 p214)

(waiting o208)
(includes o208 p51)(includes o208 p174)(includes o208 p185)

(waiting o209)
(includes o209 p76)(includes o209 p87)(includes o209 p135)(includes o209 p140)(includes o209 p173)(includes o209 p208)(includes o209 p227)

(waiting o210)
(includes o210 p43)(includes o210 p49)(includes o210 p74)(includes o210 p126)(includes o210 p146)(includes o210 p214)

(waiting o211)
(includes o211 p28)(includes o211 p59)(includes o211 p141)(includes o211 p143)(includes o211 p184)(includes o211 p206)

(waiting o212)
(includes o212 p24)(includes o212 p58)(includes o212 p82)(includes o212 p144)(includes o212 p228)

(waiting o213)
(includes o213 p17)(includes o213 p58)(includes o213 p140)(includes o213 p185)(includes o213 p203)(includes o213 p204)

(waiting o214)
(includes o214 p20)(includes o214 p68)(includes o214 p166)(includes o214 p210)(includes o214 p221)

(waiting o215)
(includes o215 p74)(includes o215 p193)

(waiting o216)
(includes o216 p30)(includes o216 p72)(includes o216 p123)(includes o216 p140)(includes o216 p188)

(waiting o217)
(includes o217 p9)(includes o217 p28)(includes o217 p112)(includes o217 p140)(includes o217 p163)(includes o217 p166)(includes o217 p191)

(waiting o218)
(includes o218 p10)(includes o218 p76)(includes o218 p81)(includes o218 p169)(includes o218 p172)(includes o218 p187)

(waiting o219)
(includes o219 p1)(includes o219 p56)(includes o219 p66)(includes o219 p75)(includes o219 p79)(includes o219 p142)(includes o219 p153)(includes o219 p172)(includes o219 p185)(includes o219 p187)(includes o219 p197)

(waiting o220)
(includes o220 p7)(includes o220 p98)(includes o220 p125)(includes o220 p146)

(waiting o221)
(includes o221 p2)(includes o221 p82)(includes o221 p144)(includes o221 p214)(includes o221 p226)

(waiting o222)
(includes o222 p104)(includes o222 p144)(includes o222 p163)(includes o222 p179)

(waiting o223)
(includes o223 p10)(includes o223 p57)(includes o223 p68)(includes o223 p137)(includes o223 p143)(includes o223 p154)(includes o223 p156)(includes o223 p160)

(waiting o224)
(includes o224 p74)(includes o224 p88)(includes o224 p124)(includes o224 p154)(includes o224 p163)(includes o224 p181)(includes o224 p210)

(waiting o225)
(includes o225 p12)(includes o225 p35)(includes o225 p55)(includes o225 p65)(includes o225 p71)(includes o225 p100)(includes o225 p148)(includes o225 p187)

(waiting o226)
(includes o226 p4)(includes o226 p12)(includes o226 p15)(includes o226 p55)(includes o226 p65)(includes o226 p103)(includes o226 p181)(includes o226 p214)

(waiting o227)
(includes o227 p41)(includes o227 p56)(includes o227 p79)(includes o227 p109)(includes o227 p116)(includes o227 p127)(includes o227 p170)(includes o227 p195)

(waiting o228)
(includes o228 p30)(includes o228 p60)(includes o228 p79)(includes o228 p128)(includes o228 p150)(includes o228 p154)

(waiting o229)
(includes o229 p1)(includes o229 p28)(includes o229 p113)(includes o229 p158)(includes o229 p173)(includes o229 p222)

(waiting o230)
(includes o230 p21)(includes o230 p68)(includes o230 p72)(includes o230 p160)(includes o230 p168)(includes o230 p169)(includes o230 p193)

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

