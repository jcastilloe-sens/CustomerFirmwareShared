//*****************************************************************************
// Test!!! Don't break my code! Code is broken anywy!
//	Customer Firmware V01.02.00
//
//	Potentiometric Breadboard
//  V6: Added Rinse mixing with buffer in calibration
// 		Use data from prefered sensors in calculations
//	V7: Started saving data in cartridge memory and having BT chip display it
//	V9: Copied from V7, added a cleaning procedure for the ORP sensor
//	V10: Commented out ORP cleaning, advanced calibration check to make sure
//		 the calibration passed as well as being within last 24 hours
//	V13: Copied from V10, this will be Deric's starting point for testing ISEs
//		 Removed pausing of small solution pockets over sensors, not measuring
//		 temperature, output ISE data over time to analyze settling
//			Updated to work with Analog V6.1
//	V14: Copied for V13, removed a lot of extraneous output added calculate
//		 two values for everything, corrected and uncorrected pvalues
//
//	Full Breadboard
//	V2: Copied from Potentiometric Breadboard V14, combine all ISE tests
//		including CO2 with amperometric testing and cleaning
//	V3: Copied from V2, added variable to control whether we're testing a die
//		with 3 NH4 sensors or CO2 sensors; Added variable to control whether we
//		store the ISEs humid or wet; Added purging the sample tube backwards
//	V4: Add variable to control whether to print raw data or not, Add variable
//		to control whether to do mixing and pump test with bubbles or not
//	V5: Calculate linearity of a line by using rinse point during calibration
//		Output extra data for linearity and TH calculations
//	Date Created: 10/1/2018
//	V6: Date Created: 10/16/2018
//		10/17/2018: Changed log k to -0.5; changed conductivity daily to read
//		low point from memory, updated factory cal box so next batch of chips
//		will have this;
//		10/23/2018: Two of three boxes would occasionally freeze, found I had
//		commented out RequestSysStatus() at beginning of calibration, believe
//		this was the problem
//		10/24/2018: Add some checks to make sure the values coming out are
//		realistic, not-negative
//		10/25/2018: Set up control loops requiring waiting in DEMO_CHECK if
//		so that demo units follow full procedure more closely; Create TH
//		postrinse variable so it can be controlled seperately from Ca+NH4
//		11/5/2018: Created continue test and continue cal commands, matched
//		to start test and start cal commands until BT and app are updated
//		11/15/2018: Changed beginning of calibration state to check next state
//		rather than SSI variable that changes with communication with BT
//		11/19/2018: Stopped clearing memory and started saving multiple cals
//		11/26/2018: Moved saving cal and test info to beginning so cartridges
//		can't be reset and keep running for more solution than they have
//		11/28/2018: Started saving ISE results not transmitted over BT
//		Added SOL_FROM_CART with logic to read solution values from memory
//	V7: Date Created: 11/3/2018
//		Removed CO2 programming (will run a different way eventually), added
//		B1 priming at beginning of test, added variable to remove Cal 3 and
//		use Rinse for conductivity calibration
//		12/17/2018: Start saving device serial number with each test
// 	V8: Date Created: 1/2/2019
//		Combined with AmpBBV28 to combine a working version of amperometric
//		testing with ISE testing
//		Measures ISE samples in large pull before mixing rather than small plug
//		Removed conductivity factory cal calculations, will only use daily cal
//		1/3/2018: Changed all LED setting to new function SetLED
//		1/7/2018: Created Init_all function to keep uniform across projects
//		1/22/2018: Added CAL_AMP_CLEANING variable to control cleaning in cal
//	V9: Date Created: 1/22/2019
//		Start saving calibration raw data, create loop so when memory runs out
//		of room starts back at beginning for both cals and tests
//		1/28/2019: Stopped outputting all currents when measuring amperometrics
//		Switched amps to not measure when not in correct range, mix up to 3 times
//		if don't find correct range continue. Save mixing numbers so next sample
//		can start at the value used for last mixing
//		Removed purging samp tube used in test case, plan on purging backwards
//		2/1/2019: Changed cleaning currents and equations to work with modified
//		analog boards, added ISE_WAIT variable
//		2/7/2019: V9.02 Started outputting ISE data overtime in both the
//		calibration and test
//		2/11/2019: V9.03 Start saving Error codes for calibrations and tests,
//		created Error Codes file to define errors that may be happening
//		2/19/2019: Scaled Cl calculation to account for dilution
//		2/25/2019: V9.04 Set up errors to be saved and displayed over BT
//	V10: Date Created: 2/27/2019
//		Adding alkalinity titration to code
//		3/7/2019: V10.01 Mixing same buffer/titrant multiple times in a row no
//		longer pulls enough sample to fill entire chip between mixings, now
//		only pulls enough to fill chip between mixings of different types
//		Adjusted sample amount used for dilution calculation
//		3/18/2018: Added priming B2 and C2 before mixing, moved all four primes
//		of small amount to right before sample purge of B1 mixing
//		Cut valve delay to 1 second instead of 2 seconds
//		Stopped following T1 mixed plug with sample, follow with air only to
//		save some sample
//	V11: Date Created: 3/20/2019
//		Changed LED scheme to blink while running, blinking blue while running
//		calibration, solid green/red at end of calibration for 1 min, pressing
//		button from idle will turn solid green if calibration is passed, solid
//		blue if calibration needs to be performed
//		4/3/2019: Started measuring temperature, added USE_MEASURED_TEMP define
//		at top of file to control using measured temperature in calculations
//		4/4/2019: Added variable Speed_ISE to both calibration and test to
//		increase pump speed while pumping bubble/air cycle and sample plug
//		4/11/2019: Got abort functionality working, app can now abort and
//		sensor will be stored in rinse (if test) or store (if cal)
//		4/22/2019: Reading Chlorine calibration for memory, if none is present
//		use values at top of main.c
//		5/1/2019: Started reading ISE 10, new die will have pH sensor there
//		5/16/2019: Put a small air bubble back in B1 and C2 pouches to prevent
//		contamination
//		5/24/2019: Added a B2 pumping cycle to calibration with a variable
//		CAL_WITH_B2 at top to control if we pump it
//		5/29/2019: Added DIE_REV_D define to control which die version we are
//		using. Moved reading NH4 for the second time from B1 to T1
//		5/30/2019: Modified alkalinity mixing so there will always be at least
//		25 steps of T1 between the two T1 mixing cycles
//		6/19/2019: Changed amperometric cleaning voltages to adjust for change
//		in pH from bicarb rinse to HEPES rinse
//		9/24/2019: Prime C2, B2, and B1 (in that order) before long sample pull
//	V13: Created: 9/26/2019
//		Minimizing air that passes over amperometrics by priming B1,B2,C2 before
//		prerinse and moving sample read/bubble cycle after chlorine measurements
//	V14: Created: 10/30/2019
//		Rearranged order of test to prime sample, read sample, read prerinse,
//		clean amps, prime buffers/conditioner, measure FCl, measure TCl,
//		measure alkalinity, post rinse
//		Changed order because V13, was reading low on pH and high on conductivity
//		which we believe was caused by leftover buffer/conditioner from chlorine
//		measurements affecting our sample read
//		Added long air purge after buffer priming in calibration to prevent buffers
//		and conditioner sitting in tubing after calibration
//	V15: Created: 11/4/2019
//		Rearranging test order to: Prime sample, Prerinse, sample ISEs, alkalinity
//		Rinse with small plug and clean amperometrics, FCl, TCl, Postrinse
//		11/25/2019: Added bubbles into every tube to prevent contamination
//		solution to solution through the valve
//		11/26/2019: For alkalinity mixing pump two plugs of T1+samp, mix with
//		first plug over sensors and second in mixing chamber, pump forward to
//		measure second plug
//	V16: Created: 12/5/2019
//		Includes updates from V15 for digital and analog boards V7, rearrange
//		calibration to do prerinse, Cal 1, Cal 2, Clean in DI+B1, prime and
//		reset B1 B2 C2 T1, Postrinse.
//		1/13/2020: Found that switches were leaving RE grounded after calibrations
//		fixed this so it leaves RE and CE floating when not using
//		1/20/2020: Moved conductivity read after ISE read, it was this way
//		previously but I was looking for conductivity problem, moving conductivity
//		read before had no impact so returned to original.
//		Added conductivity temperature compensation for calibrants when both
//		calculating slopes/intercepts and choosing range during sample, also
//		correcting sample conductivity to 25 C after using conductivity for
//		ISE calculations
//		2/6/2020: Created MeasureConductivity() function and added it to chlorine
//		mixing cycles to check that conductivity increased, if it didn't go
//		high enough set out of mixing range error,
//		4/7/2020: Added equation to calculate the drift of the reference
//	V17: Created: 4/9/2020
//		Updated for Memory_V4, chips that run V17 cannot be run on V16 and vice
//		versa, the memory will be scrambled and data may be lost
//		4/17/2020: Started cleaning and removing excess code/switches that no
//		longer apply to our testing
//		5/5/2020: Added code to pick sensors during test, will track and try
//		to improve as I go
//	V18: Created 7/16/2020
//		Updated mixing for alkalinity and both chlorines according to what Nick
//		found to be more consistent
//	V19: Created 8/13/2020
//		Added code to rerun calibration if it fails, or repump a single solution
//		if the other calibrants seemed to work
//	V20: Created 10/5/2020
//		If an ISE fails calibration, use its last passed calibration slope,
//		hoping to get decent data if a bubble ruins calibration
//	V21: Created 10/29/2020
//		Keep valve awake during pumping cycles, only sleeps when measuring
//		and while idle
//		11/5/2020: Add Potassium interference math, assume 1.0 ppm K
//	V22: Created 12/8/2020
//		Set up to run full cycle with all sensors, as well as pH+Cl cartridge
//	V23: Created 2/23/2021
//		Set up to run NH4 calibration low point in clean solution rather than
//		cal 2 if the Cal 2 pH is read from memory above 9
//	V24: Created 3/19/2021
//		Running new alkalinity algorithm that chooses steps to pump on second
//		mix based on pH of first mix. Includes new temperature correction
//		calculations that
//	V25: Created 4/6/2021
//		Redefine how far to pump based on the pumps received in 2020 as they
//		don't pump as far as the pumps received in 2018
//	V26: Created 4/9/2021
//		Modified to be compatible with the die that include hydrogen 2
//		26.3: 5/12/2021: Added nitrite to code, must be activated at top by
//		setting MEASURE_NITRITE to 1, only runs in pH_Cl cartridge
//		26.4: 5/14/2021: Fixed bugs in alkalinity
//			  5/18/2021: Moved the cartridge info to fit sensor max number of days
//		26.5: 6/2/2021: REPEAT_SAMP and REPEAT_PRERINSE defines created, calculates
//				ppm everytime sample is pumped
//	V28: Created 6/17/2021
//		Reordered calibration to pump Rinse, Clean, Cal 2, Cal 1, Repump, Postrinse
//		7/1/2021: Added functionality to measure while pumping
//		28.1: 7/14/2021: Adjusted Nitrites to have current limited clean, clean
//		10X in daily calibration, removed blank measurements on clean, increased
//		read time to 60 seconds
//		28.2: 7/26/2021: When reading pH after chlorine saves the pH values in the
//		raw data for alkalinity memory spots, and prints them into raw data
//		Prints out all reported values in Test Raw Data
//		28.3: 8/3/2021: Suspect alkalinity failing over time is because T1 is
//		getting contaminated, adjust how it sits at valve while running alkalinity
//		as well as how buffer primes are handled in calibration
//
//	V29: Created 8/10/2021
//		Updated to work with the larger memory
//		29.2: Added factory calibration to printout
//		29.3: 8/27/2021: Saving cleaning data in large memory, rebuild oxide to 1470 mV average
//		29.4: 8/27/2021: Calculate TH using both Deric's and Nick's math
//		29.5: 9/28/2021: Three second button hold will fill chip from sample vial
//		29.6: 10/6/2021: Oxide rebuild VSet is read from EEPROM, which is calibrated during QC
//		29.7: 10/12/2021: Pulling board versions off EEPROM
//		29.8: 10/25/2021: Push a plug of T2 before cleaning amperometrics
//		29.9: 10/29/2021: NH4 in high pH pumps it's own plug
//						  Modified startup procedure so it works for all electronics
//
//	Customer Firmware Created: 11/2/2021
//		V1.00.00: Created 11/2/2021: Firmware going to Denver beta test
//		V1.00.01_Testing: Created 11/2/2021: Premix Alk
//		V01.01.00 Created: 11/5/2021: Cl will mix with air over amps and reference
//		V01.01.02 Created: 11/8/2021: Created CALIBRATE_H2_IN_CLEAN predefine and set up math to calibrate in clean
//		V01.01.03 Created: 11/11/2021: Using conductivity sensor to more accurately store amps dry
//			Pushing rinse/clean plug backwards into sample tube to hopefully clean it and keep it clean
//		V01.01.04 Created 11/15/2021: Using clean as single point offset to recalculate alkalinity after it has already been ran
//		V01.01.05 Created 11/22/2021: Changed memory dump function to print all cals and tests the large memory can hold
//		V01.01.06 Created 12/1/2021: Changed TH to be calculated by iterating both Mg concentration and log K REMOVED! Using Deric's math again
//									 Saving thermistor starting and final temperatures, calculating sample temperature based off this
//		V01.01.07 Created 12/13/2021: Pumping sample multiple times looking for stability
//			12/17/2021: Turned off KEEP_VALVE_AWAKE fixed by meeting 1ms driver wake up time requirement
//		V01.01.08 Created 1/5/2022: Changed TH math to calculate a log k each calibration
//		V01.01.09 Created 1/12/2022: Updated alkalinity math to remove 30 steps from Steps T1 used to calculate alkalinity
//		V01.01.10 Created 1/20/2022: When measuring post rinse need to pump in a plug large enough to measure before storing with a bubble that separates ISEs from reference
//		V01.01.11 Created 1/26/2022: Added Predefine CALIBRATE_TH_R_2 to set whether to calibrate between Cal 1 - Cal 2 or Rinse - Cal 2
//		V01.01.12 Created 2/8/2022: Added IS_CAL_1_MG_ADJ variable so when using the new cal 1 the IS is accurate
//		V01.01.13 Created 2/8/2022: Including solution values into ISE structure, both to make changing solution values IS/K_t easier but also so memory doesn't have to be read inside functions, can just pass the structure
//		V01.01.14 Created 2/15/2022: Added IS and KT values to cartridge memory so in the future if we change these value or calibrants won't have to change code
//		V01.01.15 Created 2/23/2022: Supports across die configuration, swapped order of calibration to Rinse, Cal 2, Cal 1, Clean, Buffer prime, Rinse
//		V01.01.16 Created 3/2/2022: In alkalinity moved finding pump home position to long plug of sample before air bubble so that air bubble is smaller and always the same size
//				3/3/2022: Changed pH 6 amperometric clean to go until -2200 nA and oxide rebuild up to 1480 mV
//		V01.01.17 Created 3/16/2022: Added support for ACROSS_V2 orientation
//		V01.01.18 Created 3/30/2022: Added support for ACROSS_V3 orientation
//		V01.01.19 Created 4/15/2022: Added predefine CALIBRATE_CA_1_R, Ca slopes between rinse and Cal 2 are too high but slopes between 1 and rinse are mathcing what they should be so use this until we find out why Cal 2 is having issues
//				4/19/2022: Changed default ISE structure to across V3
//				4/21/2022: Changed valve delay in calibration to 2 seconds, added after air delay seperation at .1 s
//		V01.01.20 Created 4/28/2022: Adjust calibration pumping so when it pushes air backwards into the tube it then pumps forward again before turning, changing cal order back to Rinse, Clean, Cal 2, Cal 1, buffer prime, rinse
//		V01.01.21 Created 5/2/2022: Added support for ACROSS_V4
//					5/3/2022: Move Clean to back after Cal 1
//		V01.02.00 Created 5/10/2022: Changing pump code to run based on volume rather than steps, this will allow 2021 pump head batch to be used
//			5/31/2022: Changed Nitrfication Potential calculation to Nitrification Capacity
//		V01.02.01 Created 6/17/2022: Moved B2 prime to after C2 mix because TCl was reading low, we believe because the pH was too low for C2 conversion
//		V01.02.02 Created 6/21/2022: Changed storage after test and calibration to have the beginning part of the chip in rinse and the inlet tube in air
//			6/28/2022: Attempted to fix an alkalinity bug occurring when only one sensor landed in range on the first mix, but didn't on the second and the other did
//		V01.02.03 Created: 7/7/2022: Changed order of calibration to go Cal 2, Rinse, Clean, Cal 1, Increased Cal 1 and 2 to pumping 5 bubbles in calibration
//		V01.02.04 Created: 7/8/2022: Calibrate conductivity mid point on clean rather than rinse if its value is saved in memory
//		V01.02.05 Created: 7/14/2022: Added support for config V8 and 9, don't run conductivity in rinse if clean has solution values saved
//		V01.02.06 Created: 8/24/2022: Fixed bug in conductivity causing the calibration to fail on pH Cl cartridges, added support for AcrossV10
//		V01.02.07 Created: 9/15/2022: Added Cl errors to test so if cleaning fails both Cl mixes are skipped and FCl/TCl aren't reported
//			9/19/2022: Also updating Alk code so it always two mixes and doesn't use the assumed slope method
//			9/26/2022: Setting up Alk to require at least one mix with a pH over 3
//		V01.02.08 Created: 10/6/2022: Setting this code up to recognize Cal 3 as an option
//		V01.02.09 Created: 10/17/2022: If Ca Rinse is > 140 (hard rinse) calibrate Ca from 2 -> Rinse... basically the same as Cal 3 code...
//			10/20/2022: Change measure conductivity function to use the correct calibrants for cartridges with Cal 3
//		V01.02.10 Created: 11/1/2022: Save the ISE intercepts again, stopped saving them on V01.02.08 because I was saving all the raw data but then realized I need the intercepts for the sensor choosing functions
//		V01.02.11 Created: 11/7/2022: Want to try Cal 4 to account for pH interferences on Hardness sensors
//			11/18/2022: Turning off parameter reporting to app if necessary sensors didn't calibrate
//			11/30/2022: Last week or so added fixed pH interference for hardness sensors, 1 mV for Ca and 2 mV for TH
//			12/6/2022: Fixed if statement for storing in clean (change was in bubble in tube if statement)
//			12/7/2022: Calibration now adjusts Hardness slopes due to pH, 1 mV for Ca and 2 mV for TH
//		V01.02.12 Created: 12/22/2022: Integrated Ben's code to check the conductivity of the B1 and C2 Primes, saving these + mix conductivities to memory
//			Adjusted the voltages for pH 9 Clean
//			Changed pH 9 clean setup to calibrate both H2 and NH4 between 1 and Rinse, will need to update this to other calibrants we make if we go this way
//			Calibrants use Log K of -5 if LINEAR_PH_CORR is defined
//		V01.02.13: Created: 1/9/2023
//			Updated to work with the digital board Roam V1.0, added conductivity check on T1, on buffer prime conductivity checks moved the large pump outside of the while loop so it always happens
//		V01.02.14: Created: 1/10/2023
//			Only saving 4 bytes to Usernames and Locations names to open up space in the memory
//			1/12/2023: Adjusted calibration to work with pH 9 Clean/Cal 5/Reduced Cal 1
//		V01.02.15: Created 1/19/2023
//			Save the chosen sensors slope percentages in the memory
//			1/30/2023: Expanding Solutions structure to hold the conductivity temperature corrections, use this value rather than define
//			2/2/2023: Picking a TH sensor for the Ratio Ramp, still reporting to the app the fixed K math
//			2/13/2023: Added predefine REPORT_TH_RATIO_RAMP that will report Total Hardness, Magnesium Hardness, and Ionic Magnesium calcualated with the Ratio Ramp if defined, all other things that use TH will also
//				Pumping sample will now turn to the air port for the last few seconds to break the sample apart and avoid an antenna effect
//			2/14/2023: Updated how sensors are chosen during calibration, the code was using the wrong calibrants during the linearity check
//			2/15/2023: Fixed bug in TH math that calculated it incorrectly during runtime (but MemoryDump worked) also put a limit on the ratio so if K becomes negative it sets it to 0 and caluclates normally
//				Updated Calibration rerun code to work with pH 9 Clean/Cal 5/Cal 6 setups
//			2/17/2023: Turned off calibration postrinse measurement to reduce necessary weight of storage pouch
//			3/1/2023: Moved back where front edge of T1 priming plug ends when doing conductivity check
//			3/2/2023: Added checks for the high alkalinity range so it wouldn't keep mixing
//			3/9/2023: Added math to handle it alkalinity not adding enough acid on first mix, added checks to handle soft water
//		V01.02.16: Created 3/13/2023: Updated rev number for the alkalinity updates made previously
//			3/20/2023: Moved conductivity temperature correction to before the IS calculation because the model is based on conductivities at 25C
//			3/22/2023: Fixed bug that was causing Ionic Magnesium to not be reported
//			3/29/2023: Fixed bug in MemoryDump code that was printing out a bad conductivity when the conductivty wasn't saved and recalculating from low range
//		V01.02.17: Created 3/29/2023: Updating rev number for Ionic Magnesium fix
//		V01.02.18: Created 4/20/2023: Added ability to turn on/off FCl with the bluetooth
//			4/25/2023: Added check to Ca that stops the pH interference math at 8, below 8 it calculates it linearly
//				Added define CLEAR_SAMP_WITH_STORAGE_SOLTUION to put the storage solution into the sample tube
//				Added predefine STORE_ON_SAMP that will turn valve to sample port while storing
//				Added predefine EXPIRATION_DATE	that changes CheckCartridge function to check expiration date rather than hydration date, the first calibration will also change expiration date if cartridge lifetime needs to be shortened
//		V01.02.19: Created 5/10/2023: Switched to 10 second oxide rebuild and 90 second cathodic cleaning
//			5/30/2023: Created option to run a "cleaning" pump cycle through our chip initiated by UART
//		V01.02.20: Created 5/31/2023: Changing how calibrations are forced for customers, instead of being sent to cal state immediately it will stay in test and give
//				the app the option to still send the continue test command to move on with test, if the button is pressed or continue cal command is sent the calibration will begin
//				Not changing functionality for TESTING_MODE, it will still allow tests with the push of the button
//			6/7/2023: Changed the check on the waveform generator during calibrations
//		V01.03.00: Created 6/7/2023: Updated firmware number because this will be a customer feature release (FCl switch, Test on failed Cals)
//		V01.03.01: Created 6/8/2023: Fixed bug that wasn't turning on the blue light when a calibration was needed
//			6/8/2023: Turn on the Cl set voltage before connecting the arrays during the Cl measurement
//			6/12/2023: Decreased FCl mixing to 5 cycles with no diffusion time
//		V01.03.02: Created 6/14/2023: Changed NH4 to calibrate using the pH interference math, changed SM_NA_CAL_1 define to 0 because Cal 6 doesn't have Na
//			6/26/2023: Moved the B1 tube bubble push back in the test to after the FCl measurement rather than before
//		V01.03.03: Created 6/26/2023: Changed all the UARTprintf functions in main.c to DEBUG_PRINT macro
//		V01.03.04: Created 7/20/2023: All mixings have been set to be "short" (no diffusion time, only 5 back and forths)
//			Change measure temperature function to check if a "high precision" temp calibration is present and to adjust the old calibration if not
//			8/14/2023: Increased the acceptable ranges for conductivity calibrations
//		V01.03.05: Created 8/9/2023: Changing the conductivity calibration to include the Roam current, the slopes will be completely different so I will have to
//			update the acceptable ranges as well but this should clear up the differences between electronics
//		V01.03.06: Created 9/12/2023: During the calibration the bubbles in the tubes were being reset by pushing farther back then necessary then pulled forward,
//			going to just push back just the bubble size, new cal pouches with shorter tubes had us worried we were pushing bubbles into the pouches
//		V01.03.07: Created 9/27/2023: Fixed bug in MemoryDump function to fix failed conductivity calibrations breaking the printed out data, added UART control for the Auto calibration and in testing mode allowing the calibration to be set by day
//		V01.03.08: Created 10/11/2023: Increased the pumping delay to 6000 (from 3000) for all puming during the calibration. Having issues with bubbles getting stuck on ISEs during calibration and hoping slower puming speed will reduce this
//		V01.03.09: Created 10/13/2023: Decreased the pump speed during a test when the solution is moving over the ISEs for a reading (Rinse, Sample, Alk), but tried to leave as much fast as possible, did not slow down Cl or storage
//		V01.03.10: Created 10/17/2023: Added the ability to save the valve setup in the cartridge memory
//		V01.03.11: Created 10/31/2023: Added UART control for Cal Rerun setting
//		V01.03.12: Created 11/8/2023: Added PRIME_POUCH_TUBES that will pump bigger plugs during the first calibration with the option to run it using UART as well, this is just on testing mode only
//			11/13/2023: Changed prime to be 12cm worth of tubing (240 uL), in testing mode will be taking conductivity readings on rinse during calibration and saving them to memory
//		V01.03.13: Created 11/24/2023: Merging the EXPIRTAION_DATE and MAX_DAYS_CALS_REMAINING functionality so it is functional with V1.4 and V1.5 of app, firmware will be based on expiration date now
// 			11/27/2023: Adding define to move buffer prime from calibration to test, testing only at this point
//			Changed buffer prime to B1, T1, B2, C2
//		V01.03.14: Created 12/12/2023: Moved all calibration math and saving to before determining if a solution needs to be repumped, this allows me to determine if Cal passed
//			when deciding how big of buffer primes to do as well as display the cal data over the BT more quickly also removes the second set of calibration math so it all happens in one place
//		V01.03.15: Created 12/20/2023: Added support for disinfection cartridge, the choosing functions for pH and NH4 need to be updated, currently it will only pick out of the first 3 spots not 6 and 4 like it could
//			1/8/2024: Added update_Cartridge_Status call after rewriting expiration date so BT is updated immediately, added wrapper around CleanAmps function to skip the whole thing if abort error
//		V01.03.16: Created 1/10/2024: Changed to a universal choosing function for the ISEs, updated to be compatible with disinfection cartridge
//		V01.03.17: Created 1/22/2024: Using Rinse as the middle point for the conductivity calibration, found it is more consistent than Clean
//		V01.03.18: Created 1/30/2024: Moved all parameter calculations and saving to when they are measured so they can be reported in app ASAP
//			2/1/2024: Send commands to BT to update Test and Cal data as it progresses
//			2/7/2024: Changing low range conductivity intercpet to pass through 0, and RAV1 boards to run 1kHz first in the low range to check for DI
//			2/12/2024: Rewrite previous calibrated status after running Cl cleaning during test to update flag directly in calibrations memory
//		V01.03.19:	2/13/2024: Added UART commands to rewrite thermistor correction and Turn valve to air or sample
//		V01.03.20:	2/15/2024: Create an alternate conductivity slope for 5kHz boards to use for the 1kHz signals
//		V01.03.21:	2/19/2024: Added predefine for calibrating H2 in Cal 6 + B2 mix
//			2/21/2024: Use different conductivity factory cal slopes if running a 1 kHz board or 5 kHz board
//					Added support for different disinfection configurations
//		V01.03.22: 2/28/2024: Adding ability to specify QC points to run from UART that will red light/green light them automatically
//			3/3/2024: Adjusting H2 readings in alk mix based on the Cr reading, only if H2 reading is higher pH than Cr, as the likely cause is due to single point offset issues
//		V01.03.23: 3/11/2024: Alk when adding acid because first mix is close to endpoint calculate volume to shift pH 0.3, if first mix is in bounds but second isn't calculate to get second mix in bounds rather than just give up
//		V01.03.24: 3/12/2024: Save and print NH4 T1 mix conductivity
//		V01.03.25: 3/13/2024: H2 pH readings in alk mix correct based on Cal 6 - Rinse mV from daily cal. Added timing report
//		V01.03.26:	3/18/2024: Changed NH4 T1 mix to go if pH > 8, guess if no alkalinity is based on Ca hardness as a proxy for alkalinity, for disinfection cartridge pump 7.5 uL
//		V01.03.27: 3/20/2024: Add sample flushing step after disinfection cartridge HEPES mix
//		V01.03.28: 3/26/2024: Going back to adjusting H2 readings based on Cr sensor, adding in Cal 6+B2 H2 calibration and saving mV to memory
//		V01.03.29: 3/27/2024: Switching to storing on sample, changed NH4 T1 mix to match alkalinity mix
//*****************************************************************************
#include <stdio.h>
#include <stdint.h>
#include <stdbool.h>
#include <math.h>
#include <time.h>
#include <string.h>
#include "inc/hw_types.h"
#include "inc/hw_memmap.h"
#include "inc/hw_hibernate.h"
#include "inc/hw_gpio.h"
#include "inc/hw_i2c.h"
#include "driverlib/debug.h"
#include "driverlib/eeprom.h"
#include "driverlib/fpu.h"
#include "driverlib/gpio.h"
#include "driverlib/hibernate.h"
#include "driverlib/interrupt.h"
#include "driverlib/i2c.h"
#include "driverlib/pin_map.h"
#include "driverlib/ssi.h"
#include "driverlib/sysctl.h"
#include "driverlib/systick.h"
#include "driverlib/timer.h"
#include "driverlib/uart.h"
#include "utils/uartstdio.h"
#include "Amperometrics.h"
#include "Bluetooth.h"
#include "Communication.h"
#include "Components.h"
#include "Helper.h"
#include "Steppers.h"
#include "main.h"
#include "inc/tm4c123gh6pm.h"
#include "driverlib/adc.h"
#include "PinMap.h"

#include <stdlib.h>

#ifdef LIFETIME_TESTING
#define TIMEOUT 5000	// ms
#else
#define TIMEOUT 60000	// ms
#endif
#define RINSE_CHECK_HIGH	10
#define RINSE_CHECK_LOW		5

// Defines to control testing
#define DIE_REV_D		1		// 1 for new version of potentiometric die (Rev D, 10 spots), 0 for old version of die (9 spots + DO)
#define ENFORCE_ERRORS	1	// 0 will ignore errors and allow testing anyway, 1 will prevent device from running because of certain errors
#define ISE_WAIT	25		// Time to wait for ISEs to settle
#define PRINT_ISE_TIME_DATA		1	// 1 will print time trace data for ISEs, 0 will not print data
//#define UPDATE_BT		1
//#define STORE_IN_RINSE_T	1	// 1 will pump rinse in at the end of test, 0 will pump store in at the end of test
//#define STORE_IN_RINSE_C	1	// 1 will pump rinse in at the end of cal, 0 will pump store in at the end of cal
#define PRINT_RAW		1	// 1 will print out raw data from calibrations and tests, 0 will not print raw data
#define PURGE_SAMPLE	1	// 0 will not purge sample tube at end of test, 1 will purge sample tube backwards at end of test
#define DEMO_UNIT		0	// 1 will always show green light on test, will always show test completed (not failed) will remove double press functionality, need to go to simple_peripheral_bds to make calibration always show passed
#define SOL_FROM_CART	1	// 0 will use solution values at top of file, 1 will use solution values stored in cartridge memory
#define FLOOD_TO_STORE		1	// 1 will not pump air after final rinse plug, 0 will pump air after final rinse plug to store
#define BUBBLES_IN_TUBE		1	// 1 will push back each solution into their respective tubes to prevent solution sitting at valve and potentially contaminating, 0 will not push them back
#define PRIME_BUFFERS_CAL		1	// 1 will prime buffers and conditioner at end of calibration, 0 will skip that step
//#define PRIME_BUFFERS_TEST		// defined will do a bigger buffer prime during tests based on the number of calibrations since last cal
//#define CLEAN_COND

//#define READ_REF_DURING_COND
//#define MEASURE_LOW_COND_RANGES

#ifdef TESTING_MODE
uint8_t g_RerunCal = 1;
uint8_t g_QCSolution = 0;
#define RERUN_CALIBRATION	1 & g_RerunCal	// 1 will rerun a calibration if it fails or repump a solution if only 1 solution had a problem, 0 will just run the calibration without rerunning anything
#define PRIME_POUCH_TUBES	1	// 1 will purge pouch tubes on the first calibration, I am going to allow UART control to do it to later calibrations as well, 0 will not
uint8_t PrimePouchTubes = 0;
#else
#define RERUN_CALIBRATION	1	// 1 will rerun a calibration if it fails or repump a solution if only 1 solution had a problem, 0 will just run the calibration without rerunning anything
#define PRIME_POUCH_TUBES	1	// 1 will purge pouch tubes on the first calibration, I am going to allow UART control to do it to later calibrations as well, 0 will not
uint8_t PrimePouchTubes = 0;
#endif
//#define CHECK_NEGATIVE		0	// 1 will check that calculated values that can't be negative aren't, if they are set them to 0; 0 will output whatever is calculated

// Defines to control testing with amperometrics added
#define CAL_AMP_CLEANING	1	// 1 will clean amps twice during calibration (pre/post rinse), 0 will skip both cleanings, IF CLEANING IN ACID OR STORE CLEANING WILL ONLY HAPPEN ONCE
#define HIGH_RANGE			1	// 0 will measure low range < .5ppm, 1 will measure high range < 2ppm
#ifndef PREMIX_ALK
#define MEASURE_FCL			(1 & g_FreeCl)	// 1 will mix B1 and measure FCL, 0 will skip this mixing
#define MEASURE_TCL			(1 & g_MonoCl)	// 1 will mix B2 and measure TCL, 0 will skip this mixing
#else
#define MEASURE_FCL			0	// 1 will mix B1 and measure FCL, 0 will skip this mixing
#define MEASURE_TCL			(0 & g_MonoCl)	// 1 will mix B2 and measure TCL, 0 will skip this mixing
#endif
#define CLEAN_AMPS_TEST		1	// 0 will skip amp cleaning during tests
#define MAX_TIMES_TO_MIX	3	// # of times to attempt chlorine mixing before moving on
#define REF_DRIFT			1	// 0 will not calculate an offset for drift, any non-zero number will calculate the drift using the coefficients below
#define REF_PUMP_COEFFICIENT		0.01378
#define REF_DAYS_COEFFICIENT		0.4708

#ifdef BLEACH_POUCH
#define MEASURE_ALKALINITY		0	// Bleach pouch is taking up the T1 spot
#else
#define MEASURE_ALKALINITY		(1 & g_Alkalinity)	// 1 will mix T1 with sample and calculate alkalinity, 0 will skip this
#endif
//#define ALK_MIX_TWO_PLUGS		0	// 1 will mix two plugs of T1 + sample each time it measures, the first plug will mix over ISEs the second in mixing chamber then measure second, 0 will only mix one plug
//#define PUMP_ALK_ENDPOINT		0

//#define MEASURE_NITRITE		0	// 1 will measure nitrite, 0 will skip
#define SATURATED_KCL_REF	0	// 1 will adjust set potential for amperometrics by 1 mV for every degree C, 0 will use old method
#define CLEAN_10X_CAL		0	// 1 will clean 10x in calibration, alternating no oxide rebuild and oxide rebuild, 0 will clean once using time based cleaning

// Types of rinsing
#define REPEAT_PRERINSE			1	// MUST ALWAYS BE 1 OR GREATER!!!, a number greater than 1 will be how many times it repumps and remeasures prerinse
#define REPEAT_SAMP				1	// MUST ALWAYS BE 1 OR GREATER!!!, a number greater than 1 will be how many times it repumps and remeasures sample
//#define BACK_AND_FORTH			0	// 0 will do normal pumping, 1 will push the 3 bubbles back and forth through the channel before pumping in the plug to measure
//#define DIFFUSION_RINSE			0	// 0 will do normal pumping, 1 will pump in large plugs and place them over the sensors for a few seconds
//#define POST_BACK_AND_FORTH		0	// 0 will rinse with 1 large plug of sample after chlorines, 1 will push 3 plugs of sample back and forth through the channel and through the valve
//#define POST_DIFFUSION_RINSE	0	// 0 will rinse with 1 large plug of sample after chlorines, 1 will pump in large plugs and place them over the sensors for a few seconds
#define MEAUSURE_WHILE_PUMPING	0	// Will measure for 25 seconds then pump and measure for 1 minute, then stop and measure another 25 seconds
#define PUMP_THEN_MEASURE		1	// Will pump for a specified amount of time before stopping and taking the measurement
#define PUMP_THEN_MEASURE_REPEAT	3 // Set the max number of times to repump and check for stability, must be at least 1

#define MEASURE_NH4_T1			1	// If pH is above 8.5, will measure the plug of sample following the prime of T1 to measure NH4
#define ALK_MIX_IN_AIR			0	// 1 will pump a large air plug before metering T1 so that amps/ref are uncovered while mixing, 0 will pump small air bubble

// Defines for testing why first point is off
//#define STRETCH_SENSOR			0			// 0 will do nothing, 1 will pump a plug of cal 1 then cal 2 at the beginning of test if sensor has been sitting
//#define RESET_ELECTRONICS_READ	0	// 0 will do nothing, 1 will turn the analog board off after reading prerinse and sample then turn back on and read again
#define MEASURE_POSTRINSE		0	// 1 will measure postrinse during the test and compare to prerinse, 0 will skip
#define MEASURE_POSTRINSE_CAL	0	// 1 will measure postrinse during calibration, 0 will skip
//#define READ_CL_MIX_PH		1		// 1 will read the pH of the chlorine mix after measuring chlorine, 0 will skip
//#define MEASURE_C2_MIX		0		// 1 will read the ISEs after mixing in C2, 0 will skip this and continue mixing B2

//#define DRIVE_COUNTER_TIME		0		// 0 will not do anything, set the number of seconds to set the counter to -300 mV after sample read then pump in another plug of sample and measure again
//#define DRIVE_COUNTER_VOLTAGE	-300	// 0 will not set a voltage, DRIVE_COUNTER_TIME must have value for this to do anything

#define POWER_ANALOG_OFF	1		// 1 will turn analog board off in between tests/cals, 0 will leave analog board on as it has been
#define STORE_AT_POTENTIAL	0		// 1 will apply 300 mV to all amperometric arrays while storing them, 0 will shut off analog circuits

//#define ACID_AMP_CLEAN		0		// 1 Will pump a little acid plug through the chip before the amperometric cleaning
//#define BIAS_IN_NITRIC		0		// 1 Will bias the electrodes to -200 mV in nitric acid for 20 seconds
//#define START_IN_NITRIC		0		// 1 will pump nitric acid over arrays and let it sit for 10 minutes before asking for sample
//#define STORE_IN_NITRIC		0		// 1 will pump in nitric acid instead of rinse at the end of the test
#define STORE_HUMID			0		// 0 will flood with store at end, 1 will pump bubble over ISEs for storage
#define STORE_FRIT_DRY		1
//#define STORE_AMPS_DRY		0		// 1 will store an air bubble over amps, but keep the ISEs and reference covered
#define STORE_IN_CLEAN		0		// 1 will store in clean, 0 will store in rinse
#define STORE_PH6_CLEAN		0		// This variable will only matter if STORE_IN_CLEAN is set, will have pH only cart to store in pH 6 clean

#define CL_MIX_IN_AIR				1		// 1 will pump a bigger plug of air in between the long sample plug and cl mixed plug
#define CLEAR_SAMPLE_WITH_RINSE		1	// 1 will pump rinse backwards into sample tube to "sanitize", 0 will not
#define CLEAR_SAMP_WITH_STORAGE_SOLTUION

#define OXIDE_REBUILD_TYPE	1	// 0 will not rebuild an oxide, 1 will rebuild oxide for 10 seconds, 2 will rebuild oxide until it reaches a set mV
//#define CC_CURRENT_LIMITED		// Uncommented this will run the cathodic cleaning until a current is reached, commented out it will run for a set amount of time
#define CL_TRACE_TIME		12	// Seconds to run the Cl measurement

//// Amperometric calibration values
//float Cl_EEP_TCl_Slope = -65;
//float Cl_EEP_FCl_Slope = -65;
//float Cl_EEP_FCl_Int = 0;
//float Cl_EEP_TCl_Int = -7.0;

// Global Variables
uint8_t g_state = STATE_IDLE;
uint8_t g_next_state = STATE_IDLE;

#ifndef SOLUTION_IN_STRUCT
// HCl pouch
static float HCl_N_pouch = 0.0524;

// Rinse HEPES values for pouches made 2/28/2020
static float pH_Rinse_pouch = 7.58;
static float Ca_Rinse_pouch = 3.449;
static float TH_Rinse_pouch = 3.176;
static float NH4_Rinse_pouch = 4.263;
static float Cond_Rinse_pouch = 1036;

// Cal 2 values made 1/9/2020
static float pH_Cal_2_pouch = 7.98;
static float Ca_Cal_2_pouch = 4.034;
static float TH_Cal_2_pouch = 3.810;
//static float NH4_Cal_2_pouch = 4.898;
static float Cond_Cal_2_pouch = 318;

// Cal 1 values made 1/9/2020
static float pH_Cal_1_pouch = 6.55;
static float Ca_Cal_1_pouch = 2.738;
static float TH_Cal_1_pouch = 2.456;
static float NH4_Cal_1_pouch = 3.835;
static float Cond_Cal_1_pouch = 1655;

// Clean values
static float pH_Clean_pouch = 7.32;
static float NH4_Clean_pouch = 4.617;
#endif

//static float aTDS = 0.5;	// TDS = Conductivity * aTDS
static float T_assume = 25;	// Assume all solutions are at room temperature

////
//// Constants moved to helper.c/h
////
//float K_T_pH_Cal_1 = -0.0112; //0.0063; // Temperature coefficient Cal 1
//float K_T_pH_Cal_2 = -0.0155; //0.0053; // Temperature coefficient Cal 2
//float K_T_pH_Rinse = -0.0127;
////static float K_T_pH_Cal_3 = 0.0047; // Temperature coefficient Cal 3
////static float K_T_pH = -0.80; 	// mV/C pH temperature compensation coefficient
////static float pKa1 = 6.352;		// Constant used for alpha CO2 calculation
////static float pKa2 = 10.329;	// Constant used for aplha CO2 calculation
////static float K_T_CO2 = -0.10; 	// mV/C CO2 Temperature Compensation Coefficient
////static float K_T_Ca = 0.10;	// mV/C Calcium Temperature Compensation Coefficient
////static float K_T_TH = 0.10;	// mV/C THard Temperature Compensation Coefficient
////static float K_T_NH4 = -0.10;	// mV/C NH4 Temperature Compensation Coefficient
//float pKa_NH4 = 9.246;

//// Conductivity temperature compensation factors for calibrants (1/C) or (%/C)/100
//static float Sols->Rinse_Cond_TComp = 0.0203;
//static float Sols->Cal_1_Cond_TComp = 0.0207;
//static float Sols->Cal_2_Cond_TComp = 0.0216;
//static float Cond_TComp_Samp = 0.0198;

//void ClearMemory(void)
//{
//	// Write test data to memory right before telling BT test is completed
//	// Enter Sample Test Data 1
//	int i;
//	int k;
//
//	uint8_t Current_test = FindTestNumber();
////	uint8_t Current_cal = FindCalNumber();
//
//	uint32_t Clear_Mem = 0xFFFFFFFF;
//	for(i = 76; i < (Current_test + 1); i++)
//	{
//		for(k = 0; k < (32 * PAGES_FOR_TEST); k++)
//			MemoryWrite((PAGE_TEST + i * PAGES_FOR_TEST) - 3, (k * 4), 4, (uint8_t *) &Clear_Mem);
//	}
//
////	for(i = 1; i < (Current_cal + 1); i++)
////	{
////		for(k = 0; k < (32 * PAGES_FOR_CAL); k++)
////			MemoryWrite(Find_Cal_page(i), (k * 4), 4, (uint8_t *) &Clear_Mem);
////	}
//}

//void RunAllCondRanges(void)
//{
//	UARTprintf("Testing conductivity\n");
//
//	uint8_t Check = 0, attempt = 0;
//	while(Check != 1)
//	{
//		WaveGenSet(1);
//
//		Check = CheckCond();
//		if(attempt == 5)
//		{
//			gui32Error |= WAVE_GEN_FAIL;
//			break;
//		}
//
//		if(Check != 1)
//		{
//			InitWaveGen(1);
//			attempt++;
//		}
//	}
//
//	// Set low current range
//	// 10.7 uApp R = 309k + 499k = 808k
//	IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);
//
//	float Cond_low = ConductivityMovingAvg();
//	UARTprintf("Cond R low, Expected: 215294, Read: %d \n",  (int) Cond_low);
//
//	WaveGenSet(0);
//
//	// Set mid current range
//	// 20 uApp R = 430k
//	IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 1);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);
//
//	Check = 0;
//	while(Check != 1)
//	{
//		WaveGenSet(1);
//
//		Check = CheckCond();
//		if(attempt == 5)
//		{
//			gui32Error |= WAVE_GEN_FAIL;
//			break;
//		}
//
//		if(Check != 1)
//		{
//			InitWaveGen(1);
//			attempt++;
//		}
//	}
//
//	float Cond_mid = ConductivityMovingAvg();
//	UARTprintf("Cond R mid, Expected: 397826, Read %d \n", (int) Cond_mid);
//
//	WaveGenSet(0);
//
//	// Set high current range
//	// 45 uApp R = 180k
//	IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
//	IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);
//
//	Check = 0;
//	while(Check != 1)
//	{
//		WaveGenSet(1);
//
//		Check = CheckCond();
//		if(attempt == 5)
//		{
//			gui32Error |= WAVE_GEN_FAIL;
//			break;
//		}
//
//		if(Check != 1)
//		{
//			InitWaveGen(1);
//			attempt++;
//		}
//	}
//
//	float Cond_high = ConductivityMovingAvg();
//	UARTprintf("Cond R high, Expected: 871428, Read %d \n", (int) Cond_high);
//
//	WaveGenSet(0);
//
////	// Set Extra high current range
////	// ~72 uApp R = 95.3 kOhm
////	IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 1);
////	IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);
////
////	Check = 0;
////	while(Check != 1)
////	{
////		WaveGenSet(1);
////
////		Check = CheckCond();
////		if(attempt == 5)
////		{
////			gui32Error |= WAVE_GEN_FAIL;
////			break;
////		}
////
////		if(Check != 1)
////		{
////			InitWaveGen(1);
////			attempt++;
////		}
////	}
////
////	float Cond_Extra_high = ConductivityMovingAvg();
////	UARTprintf("Cond R Extra high, Read %d \n", (int) Cond_Extra_high);
////
////	WaveGenSet(0);
//
//	UARTprintf("Temperature: %d C \n", (int) (MeasureTemperature(1) * 1000));
//}


int main(void) {

	SysCtlClockSet(SYSCTL_SYSDIV_8 | SYSCTL_USE_PLL | SYSCTL_OSC_MAIN | SYSCTL_XTAL_16MHZ);

	// Initialize all components and steppers, find valve stepper home
	Init_all(0);

//	RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
//	TurnValveToStore();

	if(gui32Error & INITILIZATION_FAILED != 0)
	{
		// TODO: What do I do if something important failed initialization?
		PrintErrors(gui32Error, 1, STATE_IDLE);

		SetLED(BLUE_BUTTON | BLUE_BUTTON_V, 0);
		SetLED(RED_BUTTON, 1);
		while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == IO_BUTTON_PIN);
		while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == 0x00);
	}

	if(gui32Error != 0)
		PrintErrors(gui32Error, 1, STATE_IDLE);

	DEBUG_PRINT(UARTprintf("FreeCl: %d\n", MEASURE_FCL);)
	DEBUG_PRINT(UARTprintf("MonoCl: %d\n", MEASURE_TCL);)
	DEBUG_PRINT(UARTprintf("Alkalinity: %d\n", MEASURE_ALKALINITY);)

#ifdef TESTING_MODE

//	float Conductivity_reading_low = (10.76 * 0.795) / 500000;	// Current adjust this reading so the factory calibration holds across Roam units
//	MemoryWrite(PAGE_FACTORY_CAL, OFFSET_COND_READ_LOW_POINT, 4, (uint8_t *) &Conductivity_reading_low);

	if(1)
	{
//#ifdef VALVE_STRUCT
//		DEBUG_PRINT(UARTprintf("Air: %d\n", V_AIR);)
//		DEBUG_PRINT(UARTprintf("Samp: %d\n", V_SAMP);)
//		DEBUG_PRINT(UARTprintf("B1: %d\n", V_B1);)
//		DEBUG_PRINT(UARTprintf("B2: %d\n", V_B2);)
//		DEBUG_PRINT(UARTprintf("C2: %d\n", V_C2);)
//		DEBUG_PRINT(UARTprintf("Clean: %d\n", V_CLEAN);)
//		DEBUG_PRINT(UARTprintf("Cal 5: %d\n", V_CAL_2);)
//		DEBUG_PRINT(UARTprintf("Cal 6: %d\n", V_CAL_1);)
//		DEBUG_PRINT(UARTprintf("T1: %d\n", V_T1);)
//		DEBUG_PRINT(UARTprintf("Rinse: %d\n", V_RINSE);)
//#endif
		uint32_t AC;
		EEPROMRead(&AC, OFFSET_AUTO_CAL, 4);
		DEBUG_PRINT(UARTprintf("Auto Cal: %d, %d:%d\n", AC & 1, AC >> 8 & 0xFF, AC >> 16 & 0xFF);)
		DEBUG_PRINT(UARTprintf("Skipping:\n");)
		if((AC >> 1) & 1)
		{DEBUG_PRINT(UARTprintf("Sun\n");)}
		if((AC >> 2) & 1)
		{DEBUG_PRINT(UARTprintf("Mon\n");)}
		if((AC >> 3) & 1)
		{DEBUG_PRINT(UARTprintf("Tue\n");)}
		if((AC >> 4) & 1)
		{DEBUG_PRINT(UARTprintf("Wed\n");)}
		if((AC >> 5) & 1)
		{DEBUG_PRINT(UARTprintf("Thur\n");)}
		if((AC >> 6) & 1)
		{DEBUG_PRINT(UARTprintf("Fri\n");)}
		if((AC >> 7) & 1)
		{DEBUG_PRINT(UARTprintf("Sat\n");)}
		struct ISEConfig ISEs;
		FillISEStruct(&ISEs);
		DEBUG_PRINT(UARTprintf("Config: %d\n", ISEs.Config);)

//		UARTprintf("ISE Size: %d\n", sizeof(struct ISEConfig));
//		UARTprintf("Solution Size: %d\n", sizeof(struct SolutionVals));


		float Temp = MeasureTemperature(1);
		DEBUG_PRINT(UARTprintf("Current Temp: %d\n", (int) (Temp * 1000));)

		float Min_Temp = Build_float(MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_MIN_CART_TEMP, 4));
		float Max_Temp = Build_float(MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_MAX_CART_TEMP, 4));
		uint8_t * DateTime = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_MIN_TEMP_DATE, 7);
		DEBUG_PRINT(UARTprintf("Min Temp: %d at %d/%d/%d%d %d:%d\n", (int) (Min_Temp * 1000), DateTime[0], DateTime[1], DateTime[2], DateTime[3], DateTime[4], DateTime[5]);)
		DateTime = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_MAX_TEMP_DATE, 7);
		DEBUG_PRINT(UARTprintf("Max Temp: %d at %d/%d/%d%d %d:%d\n", (int) (Max_Temp * 1000), DateTime[0], DateTime[1], DateTime[2], DateTime[3], DateTime[4], DateTime[5]);)

		uint16_t Cals = *((uint16_t *) MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_COMPLETED_CALS, 2));
		uint16_t Tests = *((uint16_t *) MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_COMPLETED_TESTS, 2));
		DEBUG_PRINT(UARTprintf("Completed Cals: %d\n", Cals);)
		DEBUG_PRINT(UARTprintf("Completed Tests: %d\n", Tests);)

//		uint16_t Cal = 5;
//		uint16_t Page = Find_Cal_page(Cal);
//		uint32_t Calibration_Status = *((uint32_t *) MemoryRead(Page, OFFSET_CAL_STATUS, 4)) | 1;
//		MemoryWrite(Page, OFFSET_CAL_STATUS, 4, (uint8_t *) &Calibration_Status);

//		DEBUG_PRINT(UARTprintf("Cond Slope percent: %d\n", *((uint16_t *) MemoryRead(Page, OFFSET_COND_SLOPE_PER, 2)));)

		// Initialize floats to hold pump variables
		float PumpVolRev, PumpRatio;

		// Read from Tiva EEPROM the pump specs
		EEPROMRead((uint32_t *) &PumpVolRev, OFFSET_PUMP_VOL_PER_REV, 4);
		EEPROMRead((uint32_t *) &PumpRatio, OFFSET_PUMP_DEAD_SPOT, 4);

		DEBUG_PRINT(UARTprintf("Pump Volume: %d nL/rev\n", (int) (PumpVolRev * 1000));)
		DEBUG_PRINT(UARTprintf("Pump Ratio: %d steps/rev\n", (int) (PumpRatio * 1000));)


		float I_Low, I_Mid, I_High;
		// Read the currents off the memory, these should be saved during the QC process
		EEPROMRead((uint32_t *) &I_Low, OFFSET_COND_I_LOW, 4);
		EEPROMRead((uint32_t *) &I_Mid, OFFSET_COND_I_MID, 4);
		EEPROMRead((uint32_t *) &I_High, OFFSET_COND_I_HIGH, 4);

		if(I_Low != I_Low)
			I_Low = 10.76 * 0.795;	// Average from circuits before ARV1_0B
		if(I_Mid != I_Mid)
			I_Mid = 19.89 * 0.8;	// Average from circuits before ARV1_0B
		if(I_High != I_High)
			I_High = 43.57 * .812;	// Average from circuits before ARV1_0B

		DEBUG_PRINT(UARTprintf("Cond Low I: %d\n", (int) (I_Low * 1000));)
		DEBUG_PRINT(UARTprintf("Cond Mid I: %d\n", (int) (I_Mid * 1000));)
		DEBUG_PRINT(UARTprintf("Cond High I: %d\n", (int) (I_High * 1000));)

		// For the testing mode we are now allowing UART control for rerunning calibrations
		uint32_t holder;
		EEPROMRead(&holder, OFFSET_CAL_RERUN, 4);
		if(holder != 1 && holder != 0)
		{
			g_RerunCal = 1;
			holder = 1;
			EEPROMProgram(&holder, OFFSET_CAL_RERUN, 4);
		}
		else if(holder == 0)
			g_RerunCal = 0;
		DEBUG_PRINT(UARTprintf("Cal Re-Run: %d\n", (int) (g_RerunCal));)
	}
#endif

	ConnectMemory(1);

	uint8_t *pui8SysStatus;	// Pointer used to access data transferred from BT chip using RequestSystemStatus() function
	uint8_t Cal_Rerun = 0;	// Flag to hold if the calibration is being re-ran, defined outside of the switch/case statement to hold value when restarting cal

	PrintTime();

#ifdef LIFETIME_TESTING
	UARTprintf("Pouch Testing Code! Press button to start running!\n");
	while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == GPIO_PIN_3 && g_state == STATE_IDLE);
	while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == 0  && g_state == STATE_IDLE);
#endif

	while(1){

		switch (g_state) {

		case STATE_IDLE:
		{
			userDelay(10, 0);
			uint8_t Battery_Percent = BatteryRead(REP_SOC_REG);
			DEBUG_PRINT(UARTprintf("Battery Percent: %d\n", Battery_Percent);)

			if(Battery_Percent > 5)
			{
				// Set RE and CE floating and close RE/CE loop
				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);

				SleepValve();
			}

			SetLED(RED_BUTTON | GREEN_BUTTON | BLUE_BUTTON | RED_BUTTON_V | GREEN_BUTTON_V | BLUE_BUTTON_V, 0);

#ifdef TESTING_MODE
			g_QCSolution = 0;	// Reset this to 0 when entering idle in case it
#endif

			update_Status(STATUS_IDLE, OPERATION_IDLE);
			while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == 0);	// Wait for BT to release button right after setting status

			update_Battery(gPumping);
			g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag

			if(Battery_Percent > 5)
				RecordTemp();

			// Set hibernate timer if hibernate module is active and device is not plugged in
			if(gBoard >= V6_2 && HibernateIsActive())
			{
				if(GPIOPinRead(IO_POWER_GOOD_B_BASE, IO_POWER_GOOD_B_PIN) == IO_POWER_GOOD_B_PIN)	// Nest if statement so we don't try to read E4 on previous version of boards
				{
					TimerLoadSet64(WTIMER0_BASE, (uint64_t) SysCtlClockGet() * HIBERNATE_TIMEOUT); // Set timer for 5 minutes, if timer expires hibernate device
					TimerEnable(WTIMER0_BASE, TIMER_A);
				}
			}

			// Enable battery timer when returning to idle state
			TimerLoadSet(TIMER2_BASE, TIMER_A, SysCtlClockGet() * 60); // Set periodic timer to update battery status every minute
			TimerEnable(TIMER2_BASE, TIMER_A);

			TimerLoadSet64(WTIMER1_BASE, (uint64_t) SysCtlClockGet() * TEMP_TIMEOUT); // Set timer for 5 minutes, if timer expires hibernate device
			TimerEnable(WTIMER1_BASE, TIMER_A);

//			while(1)
//			{
//				SetLED(RED_BUTTON | GREEN_BUTTON | BLUE_BUTTON | RED_BUTTON_V | GREEN_BUTTON_V | BLUE_BUTTON_V, 0);
//				SetLED(RED_BUTTON | RED_BUTTON_V, 1);
//				userDelay(500, 0);
//				SetLED(RED_BUTTON | GREEN_BUTTON | BLUE_BUTTON | RED_BUTTON_V | GREEN_BUTTON_V | BLUE_BUTTON_V, 1);
//				userDelay(500, 0);
//				SetLED(RED_BUTTON | GREEN_BUTTON | RED_BUTTON_V | GREEN_BUTTON_V, 0);
//				userDelay(500, 0);
//			}

			if(POWER_ANALOG_OFF && gBoard >= V6_4 && STORE_AT_POTENTIAL == 0)
			{
				AnalogOff();
				//				InitAnalog();
			}
//			else if(STORE_AT_POTENTIAL)
//			{
//				DEBUG_PRINT(UARTprintf("Turning on amp arrays to 300 mV!\n");)
//
//				// Set reference for amperometric mode
//				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
//				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);
//
//				// Connect all electrodes together for measuring
//				IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 1);
//				IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);
//
//				DACVoltageSet(0, 300, true);
//			}

#ifdef LIFETIME_TESTING
			// TODO: Set command like BT told device to start
			ConnectMemory(1);

			uint16_t CurTest = FindTestNumber();
			uint16_t CurCal = FindCalNumber();
			uint8_t RunTest = 0;
			uint8_t RunCal = 0;

			if(CurCal == 0xFFFF)
				CurCal = 0;
			if(CurTest == 0xFFFF)
				CurTest = 0;


			uint8_t *Prev_Cal_Date; // Pointer to previous calibration date, read from memory
			Prev_Cal_Date = MemoryRead(Find_Cal_page(CurCal), OFFSET_CAL_DATE, 7); // Read previous cal date

			// Create time variables to convert time in human readable format to UNIX
			struct tm testTime;
			struct tm calTime;

			// Fill in time structure with data read from memory
			calTime.tm_mon = *Prev_Cal_Date - 1;	// Month [0,11]
			calTime.tm_mday = *(Prev_Cal_Date + 1); // Day [1,31]
			calTime.tm_year = (*(Prev_Cal_Date + 2) * 100) + *(Prev_Cal_Date + 3) - 1900;	// Years since 1900
			calTime.tm_hour = *(Prev_Cal_Date + 4);
			calTime.tm_min = *(Prev_Cal_Date + 5);
			calTime.tm_sec = *(Prev_Cal_Date + 6);

			UARTprintf("Cal date: %d/%d/%d \n", (int) calTime.tm_mon, (int) calTime.tm_mday, (int) calTime.tm_year);
			UARTprintf("Cal time: %d:%d:%d \n", (int) calTime.tm_hour, (int) calTime.tm_min, (int) calTime.tm_sec);

			uint8_t *Prev_Test_Date; // Pointer to previous calibration date, read from memory
			Prev_Test_Date = MemoryRead(Find_Test_page(CurTest), OFFSET_TEST_DATE, 7); // Read previous cal date

			// Fill in time structure with data read from memory
			testTime.tm_mon = *Prev_Test_Date - 1;	// Month [0,11]
			testTime.tm_mday = *(Prev_Test_Date + 1); // Day [1,31]
			testTime.tm_year = (*(Prev_Test_Date + 2) * 100) + *(Prev_Test_Date + 3) - 1900;	// Years since 1900
			testTime.tm_hour = *(Prev_Test_Date + 4);
			testTime.tm_min = *(Prev_Test_Date + 5);
			testTime.tm_sec = *(Prev_Test_Date + 6);

			UARTprintf("Test date: %d/%d/%d \n", (int) testTime.tm_mon, (int) testTime.tm_mday, (int) testTime.tm_year);
			UARTprintf("Test time: %d:%d:%d \n", (int) testTime.tm_hour, (int) testTime.tm_min, (int) testTime.tm_sec);

			uint32_t Cal_Date = mktime(&calTime);	// Get date of last cal in seconds since 1900
			uint32_t Test_Date = mktime(&testTime);	// Get date of last cal in seconds since 1900

//			// Run Tests only
//			if(CurTest < 100)
//			{
//				RunTest = 1;
//			}
//			else if(CurTest < 150 && *MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_CONFIGURATION, 1) == PH_CL_CART)
//			{
//				RunTest = 1;
//			}

			// Run a calibration then 3 tests in a row
			if(CurCal == 0)
			{
				RunCal = 1;
			}
			else if(CurTest == 0)
			{
				RunTest = 1;
			}
			else if(CurTest % 3 == 0 && CurCal < 30 && (Test_Date > Cal_Date))
			{
				RunCal = 1;
			}
			else if(CurTest < 100)
			{
//				if(CurTest == 50)
//				{
//					UARTprintf("At 50 tests! Press button to continue running!\n");
//					while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == GPIO_PIN_3);
//					while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == 0);
//				}
				RunTest = 1;
			}
			else if(CurTest < 150 && *MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_CONFIGURATION, 1) == PH_CL_CART)
			{
				RunTest = 1;
			}

#endif	//LIFETIME_TESTING




#ifdef TESTING_MODE
			// Clear UART FIFO
			while(UARTCharsAvail(UART0_BASE))
				UARTCharGetNonBlocking(UART0_BASE);

//			DEBUG_PRINT(UARTprintf("To pump sample vial through chip, type C\n");)
//			DEBUG_PRINT(UARTprintf("To turn on/off FCl, TCl, or Alk type F, T, or A\n");)

			DEBUG_PRINT(UARTprintf("UART Commands:\nA: Alkalinity\nF: Free Chlorine\nT: Total Chlorine\nP: Pump sample vial\nC: Auto Calibration\nR: Rerun Cal\nB: Prime bubbles out of pouch tubes\n");)
			DEBUG_PRINT(UARTprintf("V: Turn and store valve\nM: Write to memory\nQ: Run QC sample\n");)
			DEBUG_PRINT(UARTprintf("1: Run Factory Cal\n");)
#endif

			while(g_state == STATE_IDLE)
			{
				int Counter = 0;

#ifdef TESTING_MODE
				if(UARTCharsAvail(UART0_BASE))
				{
//					uint8_t i;
					int32_t Command = UARTCharGet(UART0_BASE);
					UARTCharPutNonBlocking(UART0_BASE, Command); //echo character
					if(Command == 'P')
					{
						uint8_t check = 1;
						DEBUG_PRINT(UARTprintf("\n");)

						DEBUG_PRINT(UARTprintf("Are you sure? (y or n)\nWarning this will not store the sensor!\n");)

						while(check == 1)
						{
							if(UARTCharsAvail(UART0_BASE))
							{
								int32_t UART_Rx = UARTCharGet(UART0_BASE);
								UARTCharPutNonBlocking(UART0_BASE, UART_Rx); //echo character

								if(UART_Rx == 'y')	// 0x0D = enter; use hex because UART_Rx is defined as int32_t because thats what UARTCharGet returns
									check = 2;
								//						else if(UART_Rx == 0x41 || UART_Rx == 0x61) // 0x41 = A, 0x61 = a
								//							CLEAN_AMPS_CHEMICAL = 1;
								else
									check = 0;
							}
						}

						if(check == 0)
						{
							DEBUG_PRINT(UARTprintf("Cancelling!\n");)
						}
						else
						{
							// Prime sample tube before test
							DEBUG_PRINT(UARTprintf("Fill sample vial with cleaning solution and press button!\n");)

							InitAnalog();
							SetLED(BLUE_BUTTON_BLINK, 1);
							while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == GPIO_PIN_3);
							while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == 0);


							DEBUG_PRINT(UARTprintf("Rinsing for 1 minutes... \n");)
//							Sensor_in_rinse = 0;
							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
//							PumpVolume(FW, PumpVol_Sample_Prime + PumpVol_sample_rinse, Speed_Fast, 1);

							// Slowly pump sample for 5 minutes
							FindPossitionZeroPump();
							PumpStepperRunTimeSpeed_AbortReady(FW, 60, 3000);
							FindPossitionZeroPump();
//							PumpStepperRunTimeSpeed_AbortReady(FW, 150, 6000);
							//				userDelay(300000, 1);

							if(PURGE_SAMPLE)
							{
								DEBUG_PRINT(UARTprintf("Purging sample tube by pushing air backwards into it\n");)
								RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
								PumpVolume(FW, 840 + 403 + 33.6, 3000, 0);
								userDelay(1, 0);
								RunValveToPossition_Bidirectional(V_SAMP, VALVE_STEPS_PER_POSITION);
								PumpVolume(BW, 403, 3000, 0);
								userDelay(1, 0);
								FindPossitionZeroPump();
							}

							TestValveDrift();

							SetLED(BLUE_BUTTON_BLINK, 0);

							if(POWER_ANALOG_OFF && gBoard >= V6_4)
							{
								AnalogOff();
							}
						}
					}

					// Simple On/Off commands
					if(Command == 'F' || Command == 'T' || Command == 'A' || Command == 'R' || Command == 'B' || Command == 'Q')
					{
						uint8_t check = 1;
						DEBUG_PRINT(UARTprintf("\n");)


						if(Command == 'F')
						{DEBUG_PRINT(UARTprintf("Type 1 to turn on FCl, type 0 to turn off FCl\n");)}
						else if(Command == 'T')
						{DEBUG_PRINT(UARTprintf("Type 1 to turn on TCl, type 0 to turn off TCl\n");)}
						else if(Command == 'A')
						{DEBUG_PRINT(UARTprintf("Type 1 to turn on Alk, type 0 to turn off Alk\n");)}
						else if(Command == 'R')
						{DEBUG_PRINT(UARTprintf("Type 1 to turn on Rerun Cal, type 0 to turn off\n");)}
						else if(Command == 'B')
						{DEBUG_PRINT(UARTprintf("Type 1 to run pouch tube priming during next Cal, anything else to cancel\n");)}
						else if(Command == 'Q')
						{DEBUG_PRINT(UARTprintf("Type 1-5 to specify which QC sample to run\n");)}

						while(check == 1)
						{
							if(UARTCharsAvail(UART0_BASE))
							{
								int32_t UART_Rx = UARTCharGet(UART0_BASE);
								UARTCharPutNonBlocking(UART0_BASE, UART_Rx); //echo character

								if(Command == 'Q' && UART_Rx >= '1' && UART_Rx <= '5')
								{
									check = 2;
									g_QCSolution = UART_Rx - '0';
									g_ui32DataRx0[0] = START_TEST;
									g_ulSSI0RXTO++;

									DEBUG_PRINT(UARTprintf("\nPress button to run QC %d\n", g_QCSolution);)
								}
								else if(UART_Rx == '0' || UART_Rx == '1')	// 0x0D = enter; use hex because UART_Rx is defined as int32_t because thats what UARTCharGet returns
								{
									check = 2;
									if(Command == 'F')
										g_FreeCl = UART_Rx - '0';
									else if(Command == 'T')
										g_MonoCl = UART_Rx - '0';
									else if(Command == 'A')
										g_Alkalinity = UART_Rx - '0';
									else if(Command == 'R')
										g_RerunCal = UART_Rx - '0';
									else if(Command == 'B')
										PrimePouchTubes = UART_Rx - '0';
									else // Catching the case where the command was Q 0 was typed
										check = 0;
								}
								else
									check = 0;
							}
						}

						if(check == 0)
						{
							DEBUG_PRINT(UARTprintf("Cancelling!\n");)
						}
						else if(check == 2)	// Check was  a valid command
						{

							if(Command == 'F')
							{
								EEPROMProgram((uint32_t *) &g_FreeCl, OFFSET_FREE_CL_CYCLE, 4);	// This gets read back in update_MonoCl() which is called in InitBT()
								DEBUG_PRINT(UARTprintf("\nFCl: %d\n", g_FreeCl);)
								update_MonoCl();
							}
							else if(Command == 'T')
							{
								EEPROMProgram((uint32_t *) &g_MonoCl, OFFSET_MONO_CL_CYCLE, 4);	// This gets read back in update_MonoCl() which is called in InitBT()
								DEBUG_PRINT(UARTprintf("\nTCl: %d\n", g_MonoCl);)
								update_MonoCl();
							}
							else if(Command == 'A')
							{
								EEPROMProgram((uint32_t *) &g_Alkalinity, OFFSET_ALKALINITY_CYCLE, 4);	// This gets read back in update_Alkalinity() which is called in InitBT()
								DEBUG_PRINT(UARTprintf("\nAlk: %d\n", g_Alkalinity);)
								update_Alkalinity();
							}
							else if(Command == 'R')
							{
								uint32_t holder = g_RerunCal;
								EEPROMProgram(&holder, OFFSET_CAL_RERUN, 4);	// This gets read back in update_MonoCl() which is called in InitBT()
								DEBUG_PRINT(UARTprintf("\nRerun Cal: %d\n", g_RerunCal);)
							}
							else if(Command == 'B')
							{
								DEBUG_PRINT(UARTprintf("\nNext cal prime pouch tubes: %d\n", PrimePouchTubes);)
							}

						}
					}

					if(Command == 'C')
					{
						uint8_t check = 0;
						DEBUG_PRINT(UARTprintf("\n");)

						DEBUG_PRINT(UARTprintf("Type 1 to turn on Auto Cal, type 0 to turn off Auto Cal\n");)
						uint32_t Auto_Cal = 0;
						uint8_t ui8Holder[4] = {0,0,0,0};

						while(check < 12)
						{
							if(UARTCharsAvail(UART0_BASE))
							{
								int32_t UART_Rx = UARTCharGet(UART0_BASE);
								UARTCharPutNonBlocking(UART0_BASE, UART_Rx); //echo character

								if(UART_Rx == '0' && check == 0)	// 0x0D = enter; use hex because UART_Rx is defined as int32_t because thats what UARTCharGet returns
								{
									check = 12;
									EEPROMProgram(&Auto_Cal, OFFSET_AUTO_CAL, 4);
								}
								else if((UART_Rx >= '0' && UART_Rx <= '9'))
								{
									if(check == 0)	// First character is on or off
									{
										DEBUG_PRINT(UARTprintf("\nType time in format HHMM, time should be in GMT! (6 hours ahead during daylight savings, 7 hour ahead otherwise)\n");)
									}
									else if(check < 5)	// Next characters are the time
									{
										ui8Holder[check - 1] = UART_Rx;	// Collect the 4 numbers in a holder array
									}
									check++;
									if(check == 5)	// Have all the data needed
									{
										DEBUG_PRINT(UARTprintf("\nDo you want to SKIP any days of the week?\n");)
										DEBUG_PRINT(UARTprintf("0-Calibrate all days\n1-Sun\n2-Mon\n3-Tues\n4-Wed\n5-Thur\n6-Fri\n7-Sat\n");)
										DEBUG_PRINT(UARTprintf("Press Enter when done\n");)

										Auto_Cal |= 1;
										Auto_Cal |= ((ui8Holder[0] - '0') * 10 + (ui8Holder[1] - '0')) << 8;
										Auto_Cal |= ((ui8Holder[2] - '0') * 10 + (ui8Holder[3] - '0')) << 16;

										EEPROMProgram(&Auto_Cal, OFFSET_AUTO_CAL, 4);
									}
									else if(check > 5)	// Now entering the days of the week
									{
										if(UART_Rx == '0')
										{
											Auto_Cal &= 0xFFFFFF01;
											EEPROMProgram(&Auto_Cal, OFFSET_AUTO_CAL, 4);
											break;
										}
										else
										{
											Auto_Cal |= 1 << (UART_Rx - '0');
										}
										EEPROMProgram(&Auto_Cal, OFFSET_AUTO_CAL, 4);
									}
								}
								else if(UART_Rx == 0x0D)	// Enter was pressed
									break;
								else
								{
									check = 0;
									break;
								}
							}
						}

						if(check == 0)
						{
							DEBUG_PRINT(UARTprintf("Cancelling!\n");)
						}
						else
						{
							DEBUG_PRINT(UARTprintf("\nAuto Cal: %d, %c%c:%c%c\n", Auto_Cal & 1, ui8Holder[0], ui8Holder[1], ui8Holder[2], ui8Holder[3]);)
							DEBUG_PRINT(UARTprintf("Skipping:\n");)
							if((Auto_Cal >> 1) & 1)
							{DEBUG_PRINT(UARTprintf("Sun\n");)}
							if((Auto_Cal >> 2) & 1)
							{DEBUG_PRINT(UARTprintf("Mon\n");)}
							if((Auto_Cal >> 3) & 1)
							{DEBUG_PRINT(UARTprintf("Tue\n");)}
							if((Auto_Cal >> 4) & 1)
							{DEBUG_PRINT(UARTprintf("Wed\n");)}
							if((Auto_Cal >> 5) & 1)
							{DEBUG_PRINT(UARTprintf("Thur\n");)}
							if((Auto_Cal >> 6) & 1)
							{DEBUG_PRINT(UARTprintf("Fri\n");)}
							if((Auto_Cal >> 7) & 1)
							{DEBUG_PRINT(UARTprintf("Sat\n");)}

							update_Auto_Cal();
						}
					}

					if(Command == 'V')
					{
						if(gCartridge == 1)
						{
							uint8_t check = 1;
							DEBUG_PRINT(UARTprintf("\n");)
							DEBUG_PRINT(UARTprintf("Type A for air, type S for sample\n");)

							uint8_t Store_Position = 0;

							while(check == 1)
							{
								if(UARTCharsAvail(UART0_BASE))
								{
									int32_t UART_Rx = UARTCharGet(UART0_BASE);
									UARTCharPutNonBlocking(UART0_BASE, UART_Rx); //echo character
									DEBUG_PRINT(UARTprintf("\n");)

									if(UART_Rx == 'A')	// 0x0D = enter; use hex because UART_Rx is defined as int32_t because thats what UARTCharGet returns
									{
										check = 2;
										DEBUG_PRINT(UARTprintf("Turning to air\n");)
										Store_Position = V_AIR;
									}
									else if(UART_Rx == 'S')
									{
										check = 2;
										DEBUG_PRINT(UARTprintf("Turning to sample\n");)
										Store_Position = V_SAMP;
									}
									else
										check = 0;

								}
							}

							if(check == 0)
							{
								DEBUG_PRINT(UARTprintf("Cancelling!\n");)
							}
							else
							{
								InitAnalog();
								SetLED(BLUE_BUTTON_BLINK, 1);


								TurnValveToStore(Store_Position);
								SetLED(BLUE_BUTTON_BLINK, 0);

								if(POWER_ANALOG_OFF && gBoard >= V6_4)
								{
									AnalogOff();
								}
							}
						}
						else
						{
							DEBUG_PRINT(UARTprintf("Plug in sensor and try again\n");)
						}
					}

					if(Command == 'M')
					{
						if(gCartridge == 1)
						{
							uint8_t check = 1;
							DEBUG_PRINT(UARTprintf("\n");)
							DEBUG_PRINT(UARTprintf("Type T to write thermistor slope\n");)

							while(check == 1)
							{
								if(UARTCharsAvail(UART0_BASE))
								{
									int32_t UART_Rx = UARTCharGet(UART0_BASE);
									UARTCharPutNonBlocking(UART0_BASE, UART_Rx); //echo character
									DEBUG_PRINT(UARTprintf("\n");)

									if(UART_Rx == 'T')	// 0x0D = enter; use hex because UART_Rx is defined as int32_t because thats what UARTCharGet returns
									{
										float Therm_corr = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_THERM_CORRECTION, 4));
										DEBUG_PRINT(UARTprintf("Current Therm slope = %d / 1000\n", (int) (Therm_corr * 1000));)

										DEBUG_PRINT(UARTprintf("Type in new slope in the format: '-0.###'\n");)
										int32_t Therm_Rx[6];
										while(check == 1)
										{
											// Clear UART FIFO
											while(UARTCharsAvail(UART0_BASE))
												UARTCharGetNonBlocking(UART0_BASE);

											uint8_t count = 0;
											for(count = 0; count < 6; count++)
											{
												Therm_Rx[count] = UARTCharGet(UART0_BASE);
												UARTCharPutNonBlocking(UART0_BASE, Therm_Rx[count]); //echo character

												if(Therm_Rx[count] == 0x0D)	// If the user pressed enter
												{
													UARTprintf("\nReceived enter, breaking for loop\n"); // Set this up so the user can press enter to restart typing the number
													break;
												}
											}
											UARTprintf("\n");

											// Check that the 4 characters are a number, followed by decimal, followed by two numbers, also check all 4 characters were recevied
											if(Therm_Rx[0] == '-' && Therm_Rx[1] == '0' && Therm_Rx[2] == '.' && (Therm_Rx[3] >= '0' && Therm_Rx[3] <= '9') && (Therm_Rx[4] >= '0' && Therm_Rx[4] <= '9') && (Therm_Rx[5] >= '0' && Therm_Rx[5] <= '9') && count == 6)
											{
												check = 2;
												Therm_corr = -(((float) (Therm_Rx[3] - '0')) / 10 + ((float) (Therm_Rx[4] - '0')) / 100 + (((float) Therm_Rx[5] - '0')) / 1000);
												DEBUG_PRINT(UARTprintf("Saving %d / 1000\n", (int) (Therm_corr * 1000));)
												MemoryWrite(PAGE_FACTORY_CAL, OFFSET_THERM_CORRECTION, 4, (uint8_t *) &Therm_corr);
											}
											else
												UARTprintf("Entered data not in required format, needs to be -0.###! Try again!\n");
										}
									}
									else
										check = 0;
								}
							}

							if(check == 0)
							{
								DEBUG_PRINT(UARTprintf("Cancelling!\n");)
							}
						}
						else
						{
							DEBUG_PRINT(UARTprintf("Plug in sensor and try again\n");)
						}
					}

					if(Command == '0')
					{
						uint8_t Device_Serial[8];
						EEPROMRead((uint32_t *) Device_Serial, OFFSET_SERIAL_NUMBER, 8);

						UARTprintf("\n");
						uint8_t i;
						for(i = 0; i < 7; i++)
							UARTCharPutNonBlocking(UART0_BASE, Device_Serial[i]); // Respond with Serial Number
						UARTprintf("\n");

					} 	// Ping SN command

					if(Command == '1')
					{
						if(POWER_ANALOG_OFF && gBoard >= V6_4 && STORE_AT_POTENTIAL == 0)
						{
							InitAnalog();
						}

						// Turn off battery timer during test
						TimerDisable(TIMER2_BASE, TIMER_A);
						TimerDisable(WTIMER0_BASE, TIMER_A);
						TimerDisable(WTIMER1_BASE, TIMER_A);

						g_state = STATE_FACTORY_CAL; // Switch to test mode
						g_next_state = STATE_IDLE;
					} 	// Factory Cal command

				}
#endif

				if(g_TimerTemperatureInterruptFlag == 1)
				{
					g_TimerTemperatureInterruptFlag = 0;
					RecordTemp();
				}

				if((g_ui32DataRx0[0] == CONTINUE_CAL || g_ui32DataRx0[0] == CONTINUE_TEST) && g_ulSSI0RXTO > 0)	// Blinks blue LED to indicate device paired to app
				{
					g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag
					update_Status(STATUS_IDLE, OPERATION_IDLE);
				}

				// Poll for SSI interuppt and blink LED instruction from BT
				if(g_ui32DataRx0[0] == BLINK_LED && g_ulSSI0RXTO > 0)	// Blinks blue LED to indicate device paired to app
				{
					g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag

					if(gBoard >= V6 && HibernateIsActive())
					{
						TimerDisable(WTIMER0_BASE, TIMER_A);
					}

					//					DEBUG_PRINT(UARTprintf("Blink LED command received! \n");)

					if(POWER_ANALOG_OFF && gBoard >= V6_4 && STORE_AT_POTENTIAL == 0)
						InitAnalog();

					if(gBoard == V6)
						GPIOPinWrite(IO_LED_FULL_INTENSITY_BASE, IO_LED_FULL_INTENSITY_PIN, IO_LED_FULL_INTENSITY_PIN); // LED full intensity

					// Use a periodic timer to blink LED
					TimerLoadSet(TIMER1_BASE, TIMER_A, SysCtlClockGet() * 0.5); // Set periodic timer
					TimerEnable(TIMER1_BASE, TIMER_A);

					uint8_t i;
					if(g_ulSSI0RXTO == 0) // Reset SSI RX interrupt flag)
					{
						for(i = 0; i < 3; i++)
						{
							while(g_TimerPeriodicInterruptFlag == 0 && g_ulSSI0RXTO == 0);
							g_TimerPeriodicInterruptFlag = 0;
							SetLED(BLUE_BUTTON | BLUE_BUTTON_V, 1);

							while(g_TimerPeriodicInterruptFlag == 0 && g_ulSSI0RXTO == 0);
							g_TimerPeriodicInterruptFlag = 0;
							SetLED(BLUE_BUTTON | BLUE_BUTTON_V, 0);
						}
					}

					TimerDisable(TIMER1_BASE, TIMER_A);
					g_TimerPeriodicInterruptFlag = 0;

					if(gBoard == V6)
						GPIOPinWrite(IO_LED_FULL_INTENSITY_BASE, IO_LED_FULL_INTENSITY_PIN, 0x00); // LED full intensity

					// Set hibernate timer if hibernate module is active and device is not plugged in
					if(gBoard >= V6_2 && HibernateIsActive())
					{
						if(GPIOPinRead(IO_POWER_GOOD_B_BASE, IO_POWER_GOOD_B_PIN) == IO_POWER_GOOD_B_PIN)	// Nest if statement so we don't try to read E4 on previous version of boards
						{
							TimerLoadSet64(WTIMER0_BASE, (uint64_t) SysCtlClockGet() * HIBERNATE_TIMEOUT); // Set timer for 5 minutes, if timer expires hibernate device
							TimerEnable(WTIMER0_BASE, TIMER_A);
						}
					}

					if(POWER_ANALOG_OFF && gBoard >= V6_4 && STORE_AT_POTENTIAL == 0)
						AnalogOff();
//					else if(STORE_AT_POTENTIAL)
//					{
//						// Set reference for amperometric mode
//						IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
//						IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);
//
//						// Connect all electrodes together for measuring
//						IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 1);
//						IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);
//
//						DACVoltageSet(0, 300, true);
//					}

				} // if blink LED command

				// Poll for SSI interuppt and instruction 0x00 from BT
#ifdef LIFETIME_TESTING
				if(RunCal)
#else
				if(g_ui32DataRx0[0] == START_CALIBRATION && g_ulSSI0RXTO > 0)
#endif	//	LIFETIME_TESTING
				{
					g_ulSSI0RXTO = 0;

					if(g_ui32DataRx0[1] == START_AUTO_CAL)
					{
						DEBUG_PRINT(UARTprintf("Auto Calibration triggered\n");)
						uint16_t MostRecentCal = FindCalNumber();
						//						uint8_t Sensor_Config = *MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_CONFIGURATION, 1);
						struct ISEConfig ISEs;
						FillISEStruct(&ISEs);

						if(MostRecentCal > 0)
						{
							pui8SysStatus = RequestSystemStatus(); // Get time and user information at beginning of calibration

							// Fill in time structure with data read from memory
							struct tm curTime;
							curTime.tm_mon = *pui8SysStatus - 1;
							curTime.tm_mday = *(pui8SysStatus + 1);
							curTime.tm_year = (*(pui8SysStatus + 2) * 100) + *(pui8SysStatus + 3) - 1900;
							curTime.tm_hour = *(pui8SysStatus + 4);
							curTime.tm_min = *(pui8SysStatus + 5);
							curTime.tm_sec = *(pui8SysStatus + 6);
							time_t CurrentTime = mktime(&curTime);	// Convert last cal time from human readable to Epoch time (seconds since 1900)

							uint8_t * calDate = MemoryRead(Find_Cal_page(MostRecentCal), OFFSET_CAL_DATE, 4);
							struct tm calTime;
							calTime.tm_mon = *calDate - 1;
							calTime.tm_mday = *(calDate + 1);
							calTime.tm_year = (*(calDate + 2) * 100) + *(calDate + 3) - 1900;
							uint8_t * pui8CalTime = MemoryRead(Find_Cal_page(MostRecentCal), OFFSET_CAL_TIME, 3);
							calTime.tm_hour = *pui8CalTime;
							calTime.tm_min = *(pui8CalTime + 1);
							calTime.tm_sec = *(pui8CalTime + 2);
							time_t CalibrationTime = mktime(&calTime);	// Convert last cal time from human readable to Epoch time (seconds since 1900)
							uint32_t Seconds_Since_Calibration = CurrentTime - CalibrationTime;

							uint32_t AC;
							EEPROMRead(&AC, OFFSET_AUTO_CAL, 4);	// Read the currently set auto-cal feature for days to skip

#ifdef TESTING_MODE
							struct tm *now = gmtime(&CurrentTime);
							if((Seconds_Since_Calibration < (3600 * 24 * ISEs.CalDelay - (3600 * 12))) ||	// For auto-calibration skip if last cal was within 12 hours of trying to run auto calibration
									((AC >> (now->tm_wday + 1)) & 1))	// Check first byte for days to skip, making this a skip variable partially for backwards compatability reasons
								break;
#else
							if(Seconds_Since_Calibration < (3600 * 24 * ISEs.CalDelay - (3600 * 12)))	// For auto-calibration skip if last cal was within 12 hours of trying to run auto calibration
								break;
#endif

						}
					}

					// Turn off interrupts during calibration
					TimerDisable(TIMER2_BASE, TIMER_A);
					TimerDisable(WTIMER1_BASE, TIMER_A);
					if(gBoard >= V6 && HibernateIsActive())
					{
						TimerDisable(WTIMER0_BASE, TIMER_A);
					}

					if(POWER_ANALOG_OFF && gBoard >= V6_4 && STORE_AT_POTENTIAL == 0)
					{
						InitAnalog();
					}

					g_state = STATE_CALIBRATION; // Switch to calibation state
					g_next_state = STATE_IDLE; // Indicate after calibration return to idle
				} // if SPI calibration instruction

				// Poll for button or SSI interrupt and instruction 0x01 from BT
#ifdef LIFETIME_TESTING
				if(RunTest)
#else
				if(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == 0 || (g_ui32DataRx0[0] == START_TEST && g_ulSSI0RXTO > 0))

#endif	// LIFETIME_TESTING
				{
					//					DEBUG_PRINT(UARTprintf("Command: %d", g_ui32DataRx0[0]);)
					//					DEBUG_PRINT(UARTprintf(" RXTO: %d\n", g_ulSSI0RXTO);)

					if(gBoard >= V6 && HibernateIsActive())
					{
						TimerDisable(WTIMER0_BASE, TIMER_A);
					}

#ifdef TESTING_MODE
					while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == 0 && Counter < 3000)
					{
						SysCtlDelay(SysCtlClockGet()/3000);
						Counter++;
					}

//					DEBUG_PRINT(UARTprintf("Air: %d\n", V_AIR);)
//					DEBUG_PRINT(UARTprintf("Samp: %d\n", V_SAMP);)
//					DEBUG_PRINT(UARTprintf("B1: %d\n", V_B1);)
//					DEBUG_PRINT(UARTprintf("B2: %d\n", V_B2);)
//					DEBUG_PRINT(UARTprintf("C2: %d\n", V_C2);)
//					DEBUG_PRINT(UARTprintf("Clean: %d\n", V_CLEAN);)
//					DEBUG_PRINT(UARTprintf("Cal 5: %d\n", V_CAL_2);)
//					DEBUG_PRINT(UARTprintf("Cal 6: %d\n", V_CAL_1);)
//					DEBUG_PRINT(UARTprintf("T1: %d\n", V_T1);)
//					DEBUG_PRINT(UARTprintf("Rinse: %d\n", V_RINSE);)

#else
					while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == 0);
#endif
					if(Counter >= 3000)	// Code to run if button was held 3 seconds
					{
												g_state = STATE_CALIBRATION;
						TimerDisable(TIMER2_BASE, TIMER_A);
						TimerDisable(WTIMER1_BASE, TIMER_A);

						if(POWER_ANALOG_OFF && gBoard >= V6_4 && STORE_AT_POTENTIAL == 0)
						{
							InitAnalog();
						}

						SetLED(BLUE_BUTTON, 1);
						while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == 0);

						break;
					}
					else
					{
						Counter = 0;

#ifdef TESTING_MODE
						if(DEMO_UNIT == 0)
						{
							while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == IO_BUTTON_PIN && Counter < 1000)
							{
								SysCtlDelay(SysCtlClockGet()/3000);
								Counter++;

								if(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == 0)
								{
									Counter = 2000;
								}
							}
						}

						if(Counter == 2000 && DEMO_UNIT == 0)	// Code to run if button was double pressed
						{
							if(POWER_ANALOG_OFF && gBoard >= V6_4 && STORE_AT_POTENTIAL == 0)
							{
								//							AnalogOff();
								InitAnalog();
							}

							//							SetLED(BLUE_BUTTON_BLINK, 1);
							//							Prime_Tubes();
							//							SetLED(BLUE_BUTTON_BLINK, 0);

							MemoryDump(0, DIE_REV_D);

							//							RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
							//							PumpStepperRunStepSpeed_AbortReady(FW, 2000, 4000);
							//							userDelay(1000, 1);
							//							RunValveToPossition_Bidirectional(V_SAMP, VALVE_STEPS_PER_POSITION);
							//							PumpStepperMix(FW, 5000, 4000, 100);
							//							PumpStepperRunStepSpeed_AbortReady(BW, 2000, 4000);
							//							userDelay(1000, 1);
							//							RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);

							g_state = STATE_IDLE;

							if(gBoard >= V6_2 && HibernateIsActive())
							{
								if(GPIOPinRead(IO_POWER_GOOD_B_BASE, IO_POWER_GOOD_B_PIN) == IO_POWER_GOOD_B_PIN)	// Nest if statement so we don't try to read E4 on previous version of boards
								{
									TimerLoadSet64(WTIMER0_BASE, (uint64_t) SysCtlClockGet() * HIBERNATE_TIMEOUT); // Set timer for 5 minutes, if timer expires hibernate device
									TimerEnable(WTIMER0_BASE, TIMER_A);
								}
							}

							if(POWER_ANALOG_OFF && gBoard >= V6_4 && STORE_AT_POTENTIAL == 0)
							{
								AnalogOff();
							}
							else if(STORE_AT_POTENTIAL)
							{
								// Set reference for amperometric mode
								IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
								IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);

								// Connect all electrodes together for measuring
								IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 1);
								IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);

								DACVoltageSet(0, 300, true);
							}

							while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == 0);
						}
						else	// Code to run if button was pressed once
#endif
						{
							if(POWER_ANALOG_OFF && gBoard >= V6_4 && STORE_AT_POTENTIAL == 0)
							{
								InitAnalog();
							}

							// Turn off battery timer during test
							TimerDisable(TIMER2_BASE, TIMER_A);
							TimerDisable(WTIMER0_BASE, TIMER_A);
							TimerDisable(WTIMER1_BASE, TIMER_A);

							g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag

//							RunAllCondRanges();

							g_state = STATE_MEASUREMENT; // Switch to test mode
							g_next_state = STATE_IDLE;

//							// Goodbye Ondrej
//							g_state = STATE_IDLE;
//							SetLED(GREEN_BUTTON_BLINK, 1);
//							PumpStepperRunTimeSpeed_AbortReady(FW, 140, 2500);
//							PumpStepperRunTimeSpeed_AbortReady(FW, 140, 2500);
//							SetLED(GREEN_BUTTON_BLINK, 0);
//							break;
						}
					}
				} // if button or SPI test instruction

			} // while idle state
			break;
		} // case
		case STATE_CALIBRATION:
		{
			// Turn on blue light while calibrating
			SetLED(BLUE_BUTTON_BLINK, 1);
			SetLED(RED_BUTTON | GREEN_BUTTON, 0);

			int counter = 0;

			// Set error to 0 at beginning
			gui32Error = 0;

			// Precheck to make sure everything is good to run calibration
			if(g_next_state != STATE_MEASUREMENT)	// Check that we are NOT coming from test, if we are don't re-ask for
				pui8SysStatus = RequestSystemStatus(); // Get time and user information at beginning of calibration
			CheckCartridge(pui8SysStatus);
			uint8_t Battery_Percent = BatteryRead(REP_SOC_REG);
			if(Battery_Percent < MIN_BAT_LEVEL && (GPIOPinRead(IO_POWER_GOOD_B_BASE, IO_POWER_GOOD_B_PIN) != 0 && gBoard >= V6_2))	// Check that battery is at minimum level or device is plugged in
			{
				DEBUG_PRINT(UARTprintf("Battery is too low! Aborting Cal!\n");)
				gui32Error |= BATTERY_TOO_LOW;
			}

			// Check for error that would prevent us from starting calibration, blink light red if this occurs
			//			if(gui32Error != 0)	// Something in the cartridge failed (max number of tests exceeded or cartridge expired) or battery check failed
			if((gui32Error & STARTING_ERRORS) != 0 || gCartridge == 0)
			{
				if(ENFORCE_ERRORS != 0)
				{
					update_Status(STATUS_CALIBRATION, OPERATION_CAL_FAILED);	// Send status to BT that calibration failed

					update_Error();
					PrintErrors(gui32Error, 1, STATE_CALIBRATION);
					if(gCartridge == 0)
						{DEBUG_PRINT(UARTprintf("No cartridge plugged in!\n");)}

					SetLED(BLUE_BUTTON, 0);
					SetLED(RED_BUTTON | RED_BUTTON_V, 1);
					while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == IO_BUTTON_PIN && counter < TIMEOUT)
					{
						SysCtlDelay(SysCtlClockGet()/3000);
						counter++;

						// Break out of while loop if continue calibration command is received
						if(g_ui32DataRx0[0] == CONTINUE_CAL && g_ulSSI0RXTO > 0)
						{
							g_ulSSI0RXTO = 0;
							break;
						}

						// Return to idle if abort command is received
						if(g_state != STATE_CALIBRATION)
							break;

					}

					g_state = STATE_IDLE;
					g_next_state = STATE_IDLE;
					break;
				}
			}

			update_Error();
			gui32Error |= (ROAM_RESET | APP_FILTER_ERROR);	// Set error to ROAM_RESET then turn off this flag at end when saving data

			if(g_next_state == STATE_MEASUREMENT)	// This if statement determines if calibration was on purpose or if calibration was expired when running test
			{
				update_Status(STATUS_CALIBRATION, OPERATION_CAL_PRECHECK);	// Send status to BT that this is pre-check

				SetLED(BLUE_BUTTON | BLUE_BUTTON_V | GREEN_BUTTON | GREEN_BUTTON_V | RED_BUTTON | RED_BUTTON_V, 0);
				SetLED(BLUE_BUTTON, 1);
				counter = 0;
				while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == IO_BUTTON_PIN && counter < TIMEOUT)
				{
					SysCtlDelay(SysCtlClockGet()/3000);
					counter++;

					// Break out of while loop if continue calibration command is received
					if(g_ui32DataRx0[0] == CONTINUE_CAL && g_ulSSI0RXTO > 0)
					{
						g_ulSSI0RXTO = 0;
						break;
					}

					if(gCartridge == 0)	// If the cartridge is unplugged return to idle
						g_state = STATE_IDLE;

					// Return to idle if abort command is received
					if(g_state != STATE_CALIBRATION)
					{
						g_next_state = STATE_IDLE;
						break;
					}
				}
				while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == 0);
				if(counter >= TIMEOUT)
					g_state = STATE_IDLE;

				SetLED(BLUE_BUTTON | BLUE_BUTTON_V | GREEN_BUTTON | GREEN_BUTTON_V | RED_BUTTON | RED_BUTTON_V, 0);
				if(g_state == STATE_CALIBRATION)
					SetLED(BLUE_BUTTON_BLINK, 1);
			}
			g_next_state = STATE_IDLE;	// Turn off this flag, used to trap when entered from trying to start a test and calibration is bad

			// Read sensor configuration from memory so program knows what type of test to run
			//			uint8_t Sensor_Config = *MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_CONFIGURATION, 1);
			struct ISEConfig ISEs;
			FillISEStruct(&ISEs);

			// Check if state changed, this happens when abort command is received
			if(g_state != STATE_CALIBRATION)
				break;

			DEBUG_PRINT(UARTprintf("\n");)
			DEBUG_PRINT(UARTprintf("Calibrating device... \n");)

			if(STORE_AT_POTENTIAL)
			{
				// Let the working electrodes float when not measuring
				IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
				IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 0);

				DACVoltageSet(0, 0, true);

				// RE and CE floating
				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);		// Leave RE floating
				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);		// Leave CE floating
			}

			//			// TODO: Set pump variables for Calibration
			// Variables to control pump and valve
			uint16_t Number_of_bubbles_Prerinse = 3;// - Rinse_pumped;
			uint16_t Number_of_bubbles_Cals = 5;
//			uint16_t Number_of_bubbles_Cal_2 = 4;
//			uint16_t Number_of_bubbles_Cal_1 = 4;
			uint16_t Number_of_bubbles_Postrinse = 2;

//			uint16_t runSteps_air_bubble = 2000;// * gPump_Ratio;			// Number of pump steps per air bubble
//			uint16_t Steps_Large_air_bubble = 5000;// * gPump_Ratio;	// 5000
//
//			uint16_t runSteps_Solution_Prerinse = 2000;// * gPump_Ratio;	// Number of pump steps for solution between air bubbles
//			uint16_t runSteps_Solution_Cal_2 = 2000;// * gPump_Ratio;		// Number of pump steps for solution between air bubbles
//			uint16_t runSteps_Solution_Cal_1 = 2000;// * gPump_Ratio;		// Number of pump steps for solution between air bubbles
//			uint16_t runSteps_Solution_Postrinse = 2000;// * gPump_Ratio;	// Number of pump steps for solution between air bubbles
//
//			uint16_t runSteps_PreRinse = (6000);// * gPump_Ratio;	// Number of pump steps to push in prerinse after air bubbles
//			uint16_t runSteps_Cal_1 = (6000);// * gPump_Ratio;		// Number of pump steps to push in Cal 1 after bubbles
//			uint16_t runSteps_Cal_2 = (6000);// * gPump_Ratio;		// Number of pump steps to push in Cal 2 after bubbles
//			uint16_t runSteps_PostRinse = (6000);// * gPump_Ratio;	// Number of pump steps to push in postrinse after air bubbles
//			uint16_t runSteps_Clean = 7000;
//			uint16_t runSteps_Clean_center = 9300;
//
//			uint16_t runSteps_plug = 8250;// * gPump_Ratio; 		// Number of pump steps to get the sample plug centered on the sensors/reference
//
//			uint16_t valve_delay = 2000;	// Delay from stopping pumping to turning valve
//			uint16_t valve_delay_after_air = 100;
//
//			// Variable to control how much air is pushed back into each tube
//			uint16_t Steps_tube_bubble = 1000;
//			uint16_t Steps_tube_prime_buffers = 2000;
//			uint16_t Steps_tube_prime = 1000;
//
//			// Variables to control store at end
//			uint16_t Steps_Store_1 = 4250;// * gPump_Ratio;
//			uint16_t Steps_Store_air_1 = 2000;// * gPump_Ratio;
//			uint16_t Steps_Store_2 = 4000;// * gPump_Ratio;
//			uint16_t Steps_Store_air_2 = 4750;// * gPump_Ratio;
//
//			uint16_t Speed_ISE = 3000;
//			uint16_t Speed_BufferPrime = 3000;
//			uint16_t Speed_Slow = 6000;	// Using this when pushing bubbles back into tubes...

			float PumpVol_air_bubble = 33.6;	// Volume uL per air bubble
			uint8_t PumpVol_Large_air_bubble = 84;	// Volume uL for large air bubbles before solution plugs

			float PumpVol_Solution = 33.6;			// Volume uL for solution between air bubbles
			float PumpVol_Solution_plug = 100.8;	// Volume uL for solution measurement plug added to PumpVol_Solution
			float PumpVol_air_plug = 141.285;	// Volume uL to get cal plugs centered on the sensors/reference

//			float PumpVol_Clean = 117.6;			// Clean plug size if pumping without measuring (pH only)
//			float PumpVol_Clean_center = 159.46;	// Volume behind air plug

			uint16_t valve_delay = 2000;			// Delay ms from stopping pumping to turning valve
			uint16_t valve_delay_after_air = 500;	// Delay ms from stopping pumping to turning valve after pumping air through valve, without fluidic resistance solution movement is almost instant

			// Variable to control how much air is pushed back into each tube
			float PumpVol_tube_bubble = 16.8;
			float PumpVol_tube_prime_buffers = 33.6;
			float PumpVol_tube_prime = 16.8;

//			// Variables to control store at end
//			float PumpVol_Store_1 = 74.09;
//			float PumpVol_Store_air_1 = 33.6;
//			float PumpVol_Store_2 = 67.2;
//			float PumpVol_Store_air_2 = 77.115;

			uint16_t Speed_ISE = 3000;
			uint16_t Speed_BufferPrime = 6000;
			uint16_t Speed_Slow = 6000;	// Using this when pushing bubbles back into tubes...

			//			uint16_t Cond_delay = 1000;

			int i;

			//
			// Collect solution data before calculations
			//
#ifdef SOLUTION_IN_STRUCT
			struct SolutionVals* Sols = FillSolutionStruct();
//			struct SolutionVals Sols;
//			FillSolutionStruct(&Sols);
#else
			float pH_EEP_Rinse, Ca_EEP_Rinse, TH_EEP_Rinse, NH4_EEP_Rinse, Cond_EEP_Rinse;
			//			float /*pH_EEP_Cal_3,*/ Cond_EEP_Cal_3;
			float pH_EEP_Cal_2, Ca_EEP_Cal_2, TH_EEP_Cal_2, /*NH4_EEP_Cal_2,*/ Cond_EEP_Cal_2;
			float pH_EEP_Cal_1, Ca_EEP_Cal_1, TH_EEP_Cal_1, NH4_EEP_Cal_1, Cond_EEP_Cal_1;
			float pH_EEP_Clean, NH4_EEP_Clean;

			pH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_PH, 4));
			if(SOL_FROM_CART == 1 && pH_EEP_Rinse == pH_EEP_Rinse)
			{
				// Rinse
				pH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_PH, 4));
				Ca_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_CA, 4));
				TH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_TH, 4));
				NH4_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_NH4, 4));
				Cond_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_COND, 4));

				// Cal 2
				pH_EEP_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_PH, 4));
				Ca_EEP_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_CA, 4));
				TH_EEP_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_TH, 4));
				//				NH4_EEP_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_NH4, 4));
				Cond_EEP_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_COND, 4));

				// Cal 1
				pH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_PH, 4));
				Ca_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_CA, 4));
				TH_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_TH, 4));
				NH4_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_NH4, 4));
				Cond_EEP_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_1_COND, 4));

				// Clean
				pH_EEP_Clean = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CLEAN_PH, 4));
				NH4_EEP_Clean = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CLEAN_NH4, 4));
			}
			else
			{
				// Rinse
				pH_EEP_Rinse = pH_Rinse_pouch;
				Ca_EEP_Rinse = Ca_Rinse_pouch;
				TH_EEP_Rinse = TH_Rinse_pouch;
				NH4_EEP_Rinse = NH4_Rinse_pouch;
				Cond_EEP_Rinse = Cond_Rinse_pouch;

				// Cal 2
				pH_EEP_Cal_2 = pH_Cal_2_pouch;
				Ca_EEP_Cal_2 = Ca_Cal_2_pouch;
				TH_EEP_Cal_2 = TH_Cal_2_pouch;
				//				NH4_EEP_Cal_2 = NH4_Cal_2_pouch;
				Cond_EEP_Cal_2 = Cond_Cal_2_pouch;

				// Cal 1
				pH_EEP_Cal_1 = pH_Cal_1_pouch;
				Ca_EEP_Cal_1 = Ca_Cal_1_pouch;
				TH_EEP_Cal_1 = TH_Cal_1_pouch;
				NH4_EEP_Cal_1 = NH4_Cal_1_pouch;
				Cond_EEP_Cal_1 = Cond_Cal_1_pouch;

				// Clean
				pH_EEP_Clean = pH_Clean_pouch;
				NH4_EEP_Clean = NH4_Clean_pouch;
			}

			// Pull K T and IS values from memory
			float IS_RINSE, IS_CLEAN, IS_CAL_1, IS_CAL_2;
			float K_T_pH_Rinse, K_T_pH_Cal_1, K_T_pH_Cal_2, K_T_pH_Clean_Sq, K_T_pH_Clean_Ln;
			IS_RINSE = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_IS_RINSE, 4));
			if(SOL_FROM_CART == 1 && IS_RINSE == IS_RINSE)
			{
				IS_CLEAN = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_IS_CLEAN, 4));
				IS_CAL_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_IS_CAL_1, 4));
				IS_CAL_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_IS_CAL_2, 4));

				K_T_pH_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_KT_RINSE, 4));
				K_T_pH_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_KT_CAL_1, 4));
				K_T_pH_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_KT_CAL_2, 4));
				K_T_pH_Clean_Sq = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_KT_CLEAN_SQ, 4));
				K_T_pH_Clean_Ln = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_KT_CLEAN_LN, 4));
			}
			else
			{
				IS_RINSE = 0.0115;
				IS_CLEAN = 0.0187;
				IS_CAL_1 = 0.0321;
				IS_CAL_2 = 0.00335;

				K_T_pH_Rinse = -0.0129;
				K_T_pH_Cal_1 = -0.0025;
				K_T_pH_Cal_2 = -0.0243;
				K_T_pH_Clean_Sq = .00007;
				K_T_pH_Clean_Ln = -.0071;
			}
#endif

//			uint8_t NH4InClean = 0;
//			if(Sols->pH_EEP_Cal_2 > 9)
			uint8_t NH4InClean = 1;	// 5/6/2022: Trying old Cal 2 formulation pH but didn't add back in NH4, so always want to measure NH4 in Clean independent of pH

			if((gui32Error & ABORT_ERRORS) != 0)
				break;

			//
			// Create record of calibration right before pumping any solution from pouches
			//
			uint8_t Zero = 0;
			uint16_t Cal_Number = FindCalNumber() + 1; // Add one to go to next free spot in memory
			//			uint8_t Cal_Number = 1;	// Fix all cals to spot 2 in memory for now, prevents erasing cal data while still allowing app to sync quickly
			uint8_t Battery_Status = BatteryRead(REP_SOC_REG);
//			float CPU_Temperature = GetCPUTemp();

			// Calculate how far off the reference has drifted
			int8_t Ref_drift = 0;
			if(REF_DRIFT != 0 && SATURATED_KCL_REF == 0)
				Ref_drift = Calculate_Ref_Drift(SATURATED_KCL_REF, 25);

			// Write calibration data to memory so BT chip can transmit it
			uint16_t Cal_page = Find_Cal_page(Cal_Number);

			for(i = 0; i < PAGES_FOR_CAL; i++)
			{
				uint8_t j;
				uint32_t Clear_mem = 0xFFFF;
				for(j = 0; j < 64; j++)
				{
					MemoryWrite(Cal_page + i, (j * 4), 1, (uint8_t *) &Clear_mem);
				}
			}

			MemoryWrite(Cal_page, OFFSET_CAL_NUMBER, 2, (uint8_t *) &Cal_Number);
			MemoryWrite(Cal_page, OFFSET_CAL_DATE, 4, pui8SysStatus);
			MemoryWrite(Cal_page, OFFSET_CAL_TIME, 3, (pui8SysStatus + 4));
//			MemoryWrite(Cal_page, OFFSET_CAL_USER, 4, (pui8SysStatus + 7));
//			MemoryWrite(Cal_page, OFFSET_CAL_GPS, 8, (pui8SysStatus + 27));
//			MemoryWrite(Cal_page, OFFSET_CAL_LOCATION, 4, (pui8SysStatus + 35));
			MemoryWrite(Cal_page, OFFSET_CAL_BATTERY, 1, &Battery_Status);
			MemoryWrite(Cal_page, OFFSET_CAL_ERROR, 4, (uint8_t *) &gui32Error);
			MemoryWrite(Cal_page, OFFSET_CAL_ZERO, 1, &Zero);

			uint32_t Calibration_Status = 0;
			MemoryWrite(Cal_page, OFFSET_CAL_STATUS, 4, (uint8_t *) &Calibration_Status);

			if(1)
			{
				uint8_t Calibrated = 0;
				uint16_t Slope_Pers[6] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
				MemoryWrite(Cal_page, OFFSET_CALIBRATED_STATUS, 1, &Calibrated);
				MemoryWrite(Cal_page, OFFSET_ALK_SLOPE_PER, 12, (uint8_t *) &Slope_Pers);
			}

#if defined REMAINING_DAYS_MAX_CALS || defined EXPIRATION_DATE
			if(Cal_Number == 1)	// If this is the first calibration, check if the expiration date needs to be adjusted
			{
				// Parse the current time from the BT data
				struct tm curTime;
				curTime.tm_mon = *pui8SysStatus - 1;
				curTime.tm_mday = *(pui8SysStatus + 1);
				curTime.tm_year = *(pui8SysStatus + 2) * 100 + *(pui8SysStatus + 3) - 1900;
				curTime.tm_hour = *(pui8SysStatus + 4);
				curTime.tm_min = *(pui8SysStatus + 5);
				curTime.tm_sec = *(pui8SysStatus + 6);

				uint32_t Date = mktime(&curTime);	// Get current time in seconds since 1900

				// Pull the hydration date from memory
				uint8_t *Sensor_Hydration;	// Pointer to data for hydration date, read from memory
				struct tm hydTime;

				Sensor_Hydration = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_HYDRATION_DATE, 4); // Read expiration date from memory

				// Fill in time structure with data read from memory
				hydTime.tm_mon = *Sensor_Hydration - 1;	// Month [0,11]
				hydTime.tm_mday = *(Sensor_Hydration + 1); // Day [1,31]
				hydTime.tm_year = (*(Sensor_Hydration + 2) * 100) + *(Sensor_Hydration + 3) - 1900;	// Years since 1900
				hydTime.tm_hour = 0;
				hydTime.tm_min = 0;
				hydTime.tm_sec = 0;

				uint32_t Hydration_Date = mktime(&hydTime);	// Convert expiration from human readable to Epoch time (seconds since 1900)

				if(Hydration_Date > Date)	// This cartridge was supposedly hydrated in the future... change hydration date to today
				{
					// Double check todays date is realistic...
					if(curTime.tm_year > 100 && curTime.tm_year < 140)	// 40 year window we are right in the middle of.. Sloppy but will fix memories that have a future hydration date
					{
						Hydration_Date = Date;

						MemoryWrite(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_HYDRATION_DATE, 4, pui8SysStatus);
					}
				}

				// Pull the max days from memory
				uint8_t Max_Days = *MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_MAX_DAYS, 1);
				if(Max_Days == 0xFF)
				{
					Max_Days = 35;	// Standard cartridge lasts for 35 days (30 + 5 for shipping)
				}

				// Pull the max Cals from memory
				uint8_t Max_Cals = *MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_MAX_CALS, 1);
				if(Max_Cals == 0xFF)
				{
					Max_Cals = 30;	// Standard cartridge lasts for 35 days (30 + 5 for shipping)
				}

				if(Hydration_Date + (Max_Days * 24 * 3600) > Date + (Max_Cals * 24 * 3600))	// Check if the currently set expiration date is further out than the current time plus cartridge lifetime
				{
					// Need to rewrite the Max Days now that cartridge's lifetime has started
					uint8_t DaysRemaining = Max_Cals + ((Date - Hydration_Date)/(24*3600));
					DEBUG_PRINT(UARTprintf("First calibration, rewriting Max Days to %d\n", DaysRemaining);)

					MemoryWrite(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_MAX_DAYS, 1, &DaysRemaining);
				}

				// Pull the expiration date from memory
				uint8_t *Sensor_Expiration;	// Pointer to data for expiration date, read from memory
				struct tm expTime;

				Sensor_Expiration = MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_EXPIRATION_DATE, 4); // Read expiration date from memory

				// Fill in time structure with data read from memory
				expTime.tm_mon = *Sensor_Expiration - 1;	// Month [0,11]
				expTime.tm_mday = *(Sensor_Expiration + 1); // Day [1,31]
				expTime.tm_year = (*(Sensor_Expiration + 2) * 100) + *(Sensor_Expiration + 3) - 1900;	// Years since 1900
				expTime.tm_hour = 0;
				expTime.tm_min = 0;
				expTime.tm_sec = 0;

				uint32_t Expiration_Date = mktime(&expTime);	// Convert expiration from human readable to Epoch time (seconds since 1900)

				if(Expiration_Date > Date + (3600 * 24 * Max_Cals))	// Check if the currently set expiration date is further out than the current time plus max cals assuming 1 cal a day
				{
					// Need to rewrite the expiration date now that cartridge's lifetime has started
					time_t New_Expiration = Date + (3600 * 24 * Max_Cals);
					struct tm *NewExp = gmtime(&New_Expiration);
					uint8_t NewExpDate[4] = {NewExp->tm_mon + 1, NewExp->tm_mday, 20, NewExp->tm_year - 100};

					DEBUG_PRINT(UARTprintf("First calibration, rewriting expiration to %d/%d/%d%d\n", NewExpDate[0], NewExpDate[1], NewExpDate[2], NewExpDate[3]);)

					MemoryWrite(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_EXPIRATION_DATE, 4, NewExpDate);
				}

				update_Cartridge_Status(gCartridge);
			}
#endif

			// EEPROM commands require multiples of 4, so create Device_Serial to be 8 long even though there are only 7 bytes necessary
			uint8_t Device_Serial[8];
			EEPROMRead((uint32_t *) Device_Serial, OFFSET_SERIAL_NUMBER, 8);
			MemoryWrite(Cal_page, OFFSET_CAL_DEVICE_SERIAL, 7, Device_Serial);

			// Save Last Passed Calibration Number, pull from last calibration
			if(Cal_Number > 1)
			{
				uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page - PAGES_FOR_CAL, OFFSET_PH_1_LAST_P_CAL, 10);
				uint8_t Last_cal_passed[10];
				memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);

				// Have to save all the values outside of MemoryRead buffer before calling MemoryWrite otherwise it could rewrite data before running check at end
				MemoryWrite(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10, Last_cal_passed);
			}

			update_Cal(Cal_Number);

//			// Save space in the memory for all 10 spots, then define where each sensor type is within that array
//			// Define Cal 2 here because NH4 is being placed in this buffer
			float ISE_mV_Cal_2[10] = {0,0,0,0,0,0,0,0,0,0};
			float *pH_H2_mV_Cal_2 = &ISE_mV_Cal_2[ISEs.pH_H2.index];
			float *pH_Cr_mV_Cal_2 = &ISE_mV_Cal_2[ISEs.pH_Cr.index];
			float *TH_mV_Cal_2 = &ISE_mV_Cal_2[ISEs.TH.index];
			float *NH4_mV_Cal_2 = &ISE_mV_Cal_2[ISEs.NH4.index];
			float *Ca_mV_Cal_2 = &ISE_mV_Cal_2[ISEs.Ca.index];

			float ISE_mV_Rinse[10] = {0,0,0,0,0,0,0,0,0,0};
			float *pH_H2_mV_Rinse = &ISE_mV_Rinse[ISEs.pH_H2.index];
			float *pH_Cr_mV_Rinse = &ISE_mV_Rinse[ISEs.pH_Cr.index];
			float *TH_mV_Rinse = &ISE_mV_Rinse[ISEs.TH.index];
			float *NH4_mV_Rinse = &ISE_mV_Rinse[ISEs.NH4.index];
			float *Ca_mV_Rinse = &ISE_mV_Rinse[ISEs.Ca.index];

			float ISE_mV_Clean[10] = {0,0,0,0,0,0,0,0,0,0};
			float *pH_H2_mV_Clean = &ISE_mV_Clean[ISEs.pH_H2.index];
			float *pH_Cr_mV_Clean = &ISE_mV_Clean[ISEs.pH_Cr.index];
			float *TH_mV_Clean = &ISE_mV_Clean[ISEs.TH.index];
			float *NH4_mV_Clean = &ISE_mV_Clean[ISEs.NH4.index];
			float *Ca_mV_Clean = &ISE_mV_Clean[ISEs.Ca.index];

			// Save space in the memory for all 10 spots, then define where each sensor type is within that array
			float ISE_mV_Cal_1[10] = {0,0,0,0,0,0,0,0,0,0};
			float *pH_H2_mV_Cal_1 = &ISE_mV_Cal_1[ISEs.pH_H2.index];
			float *pH_Cr_mV_Cal_1 = &ISE_mV_Cal_1[ISEs.pH_Cr.index];
			float *TH_mV_Cal_1 = &ISE_mV_Cal_1[ISEs.TH.index];
			float *NH4_mV_Cal_1 = &ISE_mV_Cal_1[ISEs.NH4.index];
			float *Ca_mV_Cal_1 = &ISE_mV_Cal_1[ISEs.Ca.index];

			float T_Rinse, T_Clean, T_Cal_2, T_Cal_1;
			float CalConductivityV2Low_1k = 0;
			float CalConductivityV2Low = 0, CalConductivityV1Mid = 0;
			float CalConductivityV3High = 0;
			float CalConductivityV2Mid = 0, CalConductivityV2High = 0;

#ifdef MEASURE_LOW_COND_RANGES
			float Cal_5_Low_Current, Cal_5_Mid_Current, Rinse_Low_Current, Clean_Low_Current;
			float Rinse_Cond_Mid_Raw, Rinse_Cond_High_Raw;
#endif


			uint8_t Cal_Order[5] = {V_CAL_2, V_RINSE, V_CLEAN, V_CAL_1, 0};
			uint8_t solution = 0;
			uint8_t calibrants_to_pump = 4;

			if(ISEs.Config == PH_CL_CART)
			{
				Cal_Order[0] = V_CAL_1;
				Cal_Order[1] = V_RINSE;
				Cal_Order[2] = V_CAL_2;
				Cal_Order[3] = V_CLEAN;
			}
			else if(Sols->pH_EEP_Cal_2 < 9 && Sols->Ca_EEP_Cal_1 != 0)	// This is Cal 3/Cal 4 setup
			{
				Cal_Order[0] = V_RINSE;
				Cal_Order[1] = V_CAL_2;
				Cal_Order[2] = V_CLEAN;
				Cal_Order[3] = V_CAL_1;
			}
			else if(Sols->pH_EEP_Clean > 8.5 && Sols->pH_EEP_Cal_2 < 7)	// This is pH 9 clean and Cal 5
			{
				Cal_Order[0] = V_CLEAN;
				Cal_Order[1] = V_RINSE;
				Cal_Order[2] = V_CAL_2;
				Cal_Order[3] = V_CAL_1;
			}

			for(solution = 0; solution < calibrants_to_pump; solution++)
			{
				if((gui32Error & ABORT_ERRORS) != 0)
					break;

				if(solution + 2 < OPERATION_CAL_POSTCHECK)
					update_Status(STATUS_CALIBRATION, solution + 2);

				// Putting it in a switch here inside a for loop so I can change the order for different configurations
#ifdef VALVE_STRUCT
				if(Cal_Order[solution] == V_CAL_2)
#else
				switch(Cal_Order[solution])
				{
				case V_CAL_2:
#endif
				{
					//
					// Flow Chart gray section, Cal 2
					//
					PrintTime();

					if((gui32Error & ABORT_ERRORS) == 0)
					{
//						update_Status(STATUS_CALIBRATION, OPERATION_CAL_2);
//						update_Status(STATUS_CALIBRATION, solution + 2);
#ifdef PRINT_UART
						if(Sols->pH_EEP_Cal_2 < 9 && Sols->Ca_EEP_Cal_2 < 400)
							DEBUG_PRINT(UARTprintf("Pumping Cal 5... \n");)
						else if(Sols->pH_EEP_Cal_2 < 9)
							DEBUG_PRINT(UARTprintf("Pumping Cal 3... \n");)
						else
							DEBUG_PRINT(UARTprintf("Pumping Cal 2... \n");)
#endif

						if(PRIME_POUCH_TUBES && (Cal_Number == 1 || PrimePouchTubes == 1))
						{
							DEBUG_PRINT(UARTprintf("Big Prime... \n");)
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
							FindPossitionZeroPump();
							PumpVolume(FW, PumpVol_air_bubble, 6000, 1);
							userDelay(valve_delay_after_air, 1);

							RunValveToPossition_Bidirectional_AbortReady(V_CAL_2, VALVE_STEPS_PER_POSITION);
							PumpVolume(FW, 240, 6000, 1);
							userDelay(valve_delay, 1);

							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
							PumpVolume(FW, PumpVol_air_bubble + PumpVol_Large_air_bubble, 6000, 1);
							userDelay(valve_delay_after_air, 1);
						}


						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
						FindPossitionZeroPump();
						uint16_t PumpSpeed = 3000;
						for (i = 0; i < Number_of_bubbles_Cals; i++) // Loop over air/solution cycle 3 times for single solution
						{
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
							if(i == 2)
								PumpSpeed = 6000;
							if(i == (Number_of_bubbles_Cals - 1))	// If this is the last bubble pump large bubble
								PumpVolume(FW, PumpVol_air_bubble + PumpVol_Large_air_bubble, PumpSpeed, 1);
							else
								PumpVolume(FW, PumpVol_air_bubble, PumpSpeed, 1);
							userDelay(valve_delay_after_air, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_CAL_2, VALVE_STEPS_PER_POSITION);
							if(i == 0 && BUBBLES_IN_TUBE)	// First time through loop, clear bubble from tube
								PumpVolume(FW, PumpVol_tube_prime + PumpVol_Solution, PumpSpeed, 1);
							else if(i != (Number_of_bubbles_Cals - 1))	// If this is not the last plug only pump a small solution plug
								PumpVolume(FW, PumpVol_Solution, PumpSpeed, 1);
							else	// If this is the last plug, pump the whole plug
								PumpVolume(FW, PumpVol_Solution + PumpVol_Solution_plug, PumpSpeed, 1);

							userDelay(valve_delay, 1);
						}
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						PumpVolume(FW, PumpVol_air_plug, PumpSpeed, 1);
		#ifdef KEEP_VALVE_AWAKE
						SleepValve();
		#endif
					}

					CollectISEmV(ISE_mV_Cal_2, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);

					T_Cal_2 = MeasureTemperature(1);

#ifdef READ_REF_DURING_COND
					// Measure conductivity
					// Set RE and CE floating and close RE/CE loop for conductivity
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 0);
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
#else
					// Measure conductivity
					// Set RE and CE floating and close RE/CE loop for conductivity
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
#endif



					if((gui32Error & ABORT_ERRORS) == 0)
					{
						if(Sols->pH_EEP_Cal_2 < 9.2)
						{
							ConnectMemory(0);

							uint8_t Check = 0, attempt = 0;

#ifdef MEASURE_LOW_COND_RANGES
							// Set low current range
							// 10.7 uApp R = 309k + 499k = 808k
							IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
							IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);

							Check = 0;
							attempt = 0;
							while(Check != 1)
							{
								WaveGenSet(1);

								Check = CheckCond();
								if(attempt == 5)
								{
									gui32Error |= WAVE_GEN_FAIL;
									break;
								}

								if(Check != 1)
								{
									InitWaveGen(1);
									attempt++;
								}
							}

							Cal_5_Low_Current = ConductivityMovingAvg();

							WaveGenSet(0);	// Turn off waveform generator when switching ranges

							// Set mid current range
							// 20 uApp R = 430k
							IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 1);
							IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

							Check = 0;
							attempt = 0;
							while(Check != 1)
							{
								WaveGenSet(1);

								Check = CheckCond();
								if(attempt == 5)
								{
									gui32Error |= WAVE_GEN_FAIL;
									break;
								}

								if(Check != 1)
								{
									InitWaveGen(1);
									attempt++;
								}
							}

	//						CalConductivityV2Mid = ConductivityMovingAvg();
							Cal_5_Mid_Current = ConductivityMovingAvg();

							WaveGenSet(0);	// Turn off waveform generator when switching ranges
#endif

							// Set high current range
							// 45 uApp R = 180k
							IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
							IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

							//					userDelay(Cond_delay, 1);
//							uint8_t Check = 0;
//							while(Check != 1)
//							{
//								WaveGenSet(1);
//
//								Check = CheckCond();
//								if(Check != 1)
//								{
//									InitWaveGen(1);
//									break;
//								}
//							}

							Check = 0;
							attempt = 0;
							while(Check != 1)
							{
								WaveGenSet(1);

								Check = CheckCond(COND_FREQ);
								if(attempt == 5)
								{
									gui32Error |= WAVE_GEN_FAIL;
									break;
								}

								if(Check != 1)
								{
									InitWaveGen(1, COND_FREQ);

									// Set high current range
									// 45 uApp R = 180k
									IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
									IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

									attempt++;
								}
							}

							CalConductivityV3High = ConductivityMovingAvg(COND_FREQ);



//#ifdef TESTING_MODE
//									// Set Extra high current range
//									// 72 uApp R = 95.3k
//									IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 1);
//									IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);
//
//									Check = 0;
//									attempt = 0;
//									while(Check != 1)
//									{
//										WaveGenSet(1);
//
//										Check = CheckCond();
//										if(attempt == 5)
//										{
//											gui32Error |= WAVE_GEN_FAIL;
//											break;
//										}
//
//										if(Check != 1)
//										{
//											InitWaveGen(1);
//											attempt++;
//										}
//									}
//
//									DEBUG_PRINT(UARTprintf("Extra High Range: %d nA\n", (int) (ConductivityMovingAvg() * 1000));)
//#endif



							WaveGenSet(0);	// Turn off waveform generator
						}
						else
						{
							ConnectMemory(0);

							if(gABoard >= ARV1_0B)
							{
								InitWaveGen(0, 1000);	// Change frequency to 1kHz

								// Set low current range
								// 10.7 uApp R = 309k + 499k = 808k
								IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
								IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);

								uint8_t Check = 0, attempt = 0;
								while(Check != 1)
								{
									WaveGenSet(1);

									Check = CheckCond(1000);
									if(attempt == 5)
									{
										gui32Error |= WAVE_GEN_FAIL;
										break;
									}

									if(Check != 1)
									{
										InitWaveGen(1, 1000);
										attempt++;
									}
								}

								CalConductivityV2Low_1k = ConductivityMovingAvg(1000);

								WaveGenSet(0);	// Turn off waveform generator when switching ranges

								InitWaveGen(0, 5000);	// Change frequency back to 5kHz
							}

							// Set low current range
							// 10.7 uApp R = 309k + 499k = 808k
							IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
							IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);

							//					userDelay(Cond_delay, 1);
//							uint8_t Check = 0;
//							while(Check != 1)
//							{
//								WaveGenSet(1);
//
//								Check = CheckCond();
//								if(Check != 1)
//								{
//									InitWaveGen(1);
//									break;
//								}
//							}

							uint8_t Check = 0, attempt = 0;
							while(Check != 1)
							{
								WaveGenSet(1);

								Check = CheckCond(COND_FREQ);
								if(attempt == 5)
								{
									gui32Error |= WAVE_GEN_FAIL;
									break;
								}

								if(Check != 1)
								{
									InitWaveGen(1, COND_FREQ);
									attempt++;
								}
							}

							CalConductivityV2Low = ConductivityMovingAvg(COND_FREQ);

							WaveGenSet(0);	// Turn off waveform generator when switching ranges

							// Set mid current range
							// 20 uApp R = 430k
							IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 1);
							IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

							//					userDelay(Cond_delay, 1);
//							Check = 0;
//							while(Check != 1)
//							{
//								WaveGenSet(1);
//
//								Check = CheckCond();
//								if(Check != 1)
//								{
//									InitWaveGen(1);
//									break;
//								}
//							}

							Check = 0;
							attempt = 0;
							while(Check != 1)
							{
								WaveGenSet(1);

								Check = CheckCond(COND_FREQ);
								if(attempt == 5)
								{
									gui32Error |= WAVE_GEN_FAIL;
									break;
								}

								if(Check != 1)
								{
									InitWaveGen(1, COND_FREQ);

									// Set mid current range
									// 20 uApp R = 430k
									IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 1);
									IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

									attempt++;
								}
							}

							CalConductivityV1Mid = ConductivityMovingAvg(COND_FREQ);

							WaveGenSet(0);	// Turn off waveform generator
						}
					}

					ConnectMemory(1);
					MemoryWrite(Cal_page, OFFSET_CR_CAL_2_MV, 40, (uint8_t *) ISE_mV_Cal_2);

					update_Cal(Cal_Number);

					// Push air back into Cal 2 port before moving to next solution
					if(BUBBLES_IN_TUBE)
					{
						RunValveToPossition_Bidirectional_AbortReady(V_CAL_2, VALVE_STEPS_PER_POSITION);
						//				PumpStepperRunStepSpeed_AbortReady(BW, Steps_tube_bubble, Speed_ISE);
						//				userDelay(valve_delay_after_air, 1);
						PumpVolume(BW, PumpVol_tube_bubble /*+ 13.77*/, Speed_Slow, 1);
//						PumpVolume(FW, 13.77, Speed_Slow, 1);
						userDelay(valve_delay, 1);
					}

#ifndef VALVE_STRUCT	// Breaks are required to prevent running into the next soltu9ion if using case/switch, but not if using if/else if
					break;
#endif
				}	// V_CAL_2
#ifdef VALVE_STRUCT
				else if(Cal_Order[solution] == V_RINSE)
#else
				case V_RINSE:
#endif
				{
					//
					// Flow chart teal section, ISE post check, PreRinse
					//
					PrintTime();

					if((gui32Error & ABORT_ERRORS) == 0)
					{
//						update_Status(STATUS_CALIBRATION,OPERATION_CAL_RINSE);
//						update_Status(STATUS_CALIBRATION, solution + 2);
						DEBUG_PRINT(UARTprintf("Pumping Prerinse... \n");)

						if(PRIME_POUCH_TUBES && (Cal_Number == 1 || PrimePouchTubes == 1))
						{
							DEBUG_PRINT(UARTprintf("Big Prime... \n");)
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
							FindPossitionZeroPump();
							PumpVolume(FW, PumpVol_air_bubble, 6000, 1);
							userDelay(valve_delay_after_air, 1);

							RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);
							PumpVolume(FW, 240, 6000, 1);
							userDelay(valve_delay, 1);

							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
							PumpVolume(FW, PumpVol_air_bubble + PumpVol_Large_air_bubble, 6000, 1);
							userDelay(valve_delay_after_air, 1);
						}

						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
						FindPossitionZeroPump();	// Find zero position of pump at beginning of every pump cycle
						uint16_t PumpSpeed = 3000;
						for (i = 0; i < Number_of_bubbles_Prerinse; i++) // Loop over air/solution cycle 3 times for single solution
						{
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
							if(i == 2)
								PumpSpeed = 6000;
							if(i == (Number_of_bubbles_Prerinse - 1))
								PumpVolume(FW, PumpVol_air_bubble + PumpVol_Large_air_bubble, PumpSpeed, 1);
							else
								PumpVolume(FW, PumpVol_air_bubble, PumpSpeed, 1);
							userDelay(valve_delay_after_air, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);
							if(i == 0 && BUBBLES_IN_TUBE)	// First time through loop, clear bubble from tube
								PumpVolume(FW, PumpVol_tube_prime + PumpVol_Solution, PumpSpeed, 1);
							else if(i != (Number_of_bubbles_Prerinse - 1))	// If this is not the last plug only pump a small solution plug
								PumpVolume(FW, PumpVol_Solution, PumpSpeed, 1);
							else	// If this is the last plug, pump the whole plug
								PumpVolume(FW, PumpVol_Solution + PumpVol_Solution_plug, PumpSpeed, 1);

							userDelay(valve_delay, 1);
						}
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						PumpVolume(FW, PumpVol_air_plug, PumpSpeed, 1);
		#ifdef KEEP_VALVE_AWAKE
						SleepValve();
		#endif
					}

					CollectISEmV(ISE_mV_Rinse, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);

					// Measure Temperature for 3 seconds
					T_Rinse = MeasureTemperature(1);
					//			float T_Rinse = T_assume;

#ifdef READ_REF_DURING_COND
					// Measure conductivity
					// Set RE and CE floating and close RE/CE loop for conductivity
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 0);
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
#else
					// Measure conductivity
					// Set RE and CE floating and close RE/CE loop for conductivity
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
#endif

//#ifdef TESTING_MODE
					if((gui32Error & ABORT_ERRORS) == 0)
//#else
//					if((gui32Error & ABORT_ERRORS) == 0 && (Sols->Cond_EEP_Clean != Sols->Cond_EEP_Clean || ISEs.Config == PH_CL_CART))
//#endif

					{
						ConnectMemory(0);

						uint8_t Check = 0, attempt = 0;

#ifdef MEASURE_LOW_COND_RANGES
							// Set low current range
							// 10.7 uApp R = 309k + 499k = 808k
							IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
							IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);

							//					userDelay(Cond_delay, 1);
//							uint8_t Check = 0;
//							while(Check != 1)
//							{
//								WaveGenSet(1);
//
//								Check = CheckCond();
//								if(Check != 1)
//								{
//									InitWaveGen(1);
////									break;
//								}
//							}

							Check = 0;
							attempt = 0;
							while(Check != 1)
							{
								WaveGenSet(1);

								Check = CheckCond();
								if(attempt == 5)
								{
									gui32Error |= WAVE_GEN_FAIL;
									break;
								}

								if(Check != 1)
								{
									InitWaveGen(1);
									attempt++;
								}
							}

							Rinse_Low_Current = ConductivityMovingAvg();

							WaveGenSet(0);	// Turn off waveform generator when switching ranges
#else
							float Rinse_Cond_Mid_Raw, Rinse_Cond_High_Raw;
#endif


						// Set mid current range
						// 20 uApp R = 430k
						IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 1);
						IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

						Check = 0;
						attempt = 0;
						while(Check != 1)
						{
							WaveGenSet(1);

							Check = CheckCond(COND_FREQ);
							if(attempt == 5)
							{
								gui32Error |= WAVE_GEN_FAIL;
								break;
							}

							if(Check != 1)
							{
								InitWaveGen(1, COND_FREQ);

								// Set mid current range
								// 20 uApp R = 430k
								IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 1);
								IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

								attempt++;
							}
						}

						CalConductivityV2Mid = ConductivityMovingAvg(COND_FREQ);
//						Rinse_Cond_Mid_Raw = ConductivityMovingAvg();

						WaveGenSet(0);	// Turn off waveform generator when switching ranges

						// Set high current range
						// 45 uApp R = 180k
						IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
						IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

						//					userDelay(Cond_delay, 1);
//						Check = 0;
//						while(Check != 1)
//						{
//							WaveGenSet(1);
//
//							Check = CheckCond();
//							if(Check != 1)
//							{
//								InitWaveGen(1);
////								break;
//							}
//						}

						Check = 0;
						attempt = 0;
						while(Check != 1)
						{
							WaveGenSet(1);

							Check = CheckCond(COND_FREQ);
							if(attempt == 5)
							{
								gui32Error |= WAVE_GEN_FAIL;
								break;
							}

							if(Check != 1)
							{
								InitWaveGen(1, COND_FREQ);

								// Set high current range
								// 45 uApp R = 180k
								IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
								IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

								attempt++;
							}
						}

						CalConductivityV2High = ConductivityMovingAvg(COND_FREQ);
//						Rinse_Cond_High_Raw = ConductivityMovingAvg();

						WaveGenSet(0);	// Turn off waveform generator

//						if((Sols->Cond_EEP_Clean != Sols->Cond_EEP_Clean || ISEs.Config == PH_CL_CART))
//						{
//							CalConductivityV2Mid = Rinse_Cond_Mid_Raw;
//							CalConductivityV2High = Rinse_Cond_High_Raw;
//						}

						ConnectMemory(1);

						// Current correct values before displaying
						float I_Mid, I_High;

						// Read the currents off the memory, these should be saved during the QC process
						EEPROMRead((uint32_t *) &I_Mid, OFFSET_COND_I_MID, 4);
						EEPROMRead((uint32_t *) &I_High, OFFSET_COND_I_HIGH, 4);

						if(I_Mid != I_Mid)
							I_Mid = 19.89 * 0.8;	// Average from circuits before ARV1_0B
						if(I_High != I_High)
							I_High = 43.57 * .812;	// Average from circuits before ARV1_0B

						Rinse_Cond_Mid_Raw = (1000000 * I_Mid) / CalConductivityV2Mid;
						Rinse_Cond_High_Raw = (1000000 * I_High) / CalConductivityV2High;

						MemoryWrite(Cal_page, OFFSET_RINSE_MID_RAW, 4, (uint8_t *) &Rinse_Cond_Mid_Raw);
						MemoryWrite(Cal_page, OFFSET_RINSE_HIGH_RAW, 4, (uint8_t *) &Rinse_Cond_High_Raw);
					}

					if(ISEs.Config == PH_CL_CART)
					{
//						update_Status(STATUS_CALIBRATION, OPERATION_FCL_ACTIVATION);
//						update_Status(STATUS_CALIBRATION, solution + 2);

						if(REF_DRIFT != 0 && SATURATED_KCL_REF != 0)
							Ref_drift = Calculate_Ref_Drift(SATURATED_KCL_REF, T_Rinse);
//						if(CLEAN_10X_CAL)
//						{
//							for(i = 0; i < 5; i++)
//							{
//								CleanAmperometrics_CurrentLimited(Ref_drift, Cal_Number, 0, 0);
//								CleanAmperometrics_CurrentLimited(Ref_drift, Cal_Number, 0, 1);
//							}
//						}
//						else

						gui32Error &= ~(CL_CLEANING_OUT_OF_RANGE);	// Clear the Cl cleaning failed flag in case we are repumping clean
#ifndef CV_CLEANING
#ifdef CC_CURRENT_LIMITED
						CleanAmperometrics_CurrentLimited(Ref_drift, Cal_Number, 0, OXIDE_REBUILD_TYPE);
#else
						CleanAmperometrics(Ref_drift, Cal_Number, 0, OXIDE_REBUILD_TYPE);
#endif
#else
						RunCVCleaningCycle(Ref_drift, Cal_Number, 0);
#endif
					}

					ConnectMemory(1);
					MemoryWrite(Cal_page, OFFSET_CR_ISE_1_RINSE, 40, (uint8_t *) ISE_mV_Rinse);

					update_Cal(Cal_Number);

					// Push air back into rinse port before moving to next solution
					if(BUBBLES_IN_TUBE)
					{
						RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);
						PumpVolume(BW, PumpVol_tube_bubble /*+ 13.77*/, Speed_Slow, 1);
//						PumpVolume(FW, 13.77, Speed_Slow, 1);
						userDelay(valve_delay, 1);
					}

#ifndef VALVE_STRUCT	// Breaks are required to prevent running into the next soltu9ion if using case/switch, but not if using if/else if
					break;
#endif
				}	// case V_RINSE
#ifdef VALVE_STRUCT
				else if(Cal_Order[solution] == V_CLEAN)
#else
				case V_CLEAN:
#endif
				{
					//
					// Clean amperometrics in cleaning solution
					//
					if(NH4InClean)
					{
						PrintTime();
						if((gui32Error & ABORT_ERRORS) == 0)
							if(CAL_AMP_CLEANING == 1 && ISEs.Config != PH_CL_CART)
							{
								PrintTime();
								if((gui32Error & ABORT_ERRORS) == 0)
								{
//									update_Status(STATUS_CALIBRATION, OPERATION_FCL_ACTIVATION);
//									update_Status(STATUS_CALIBRATION, solution + 2);
									DEBUG_PRINT(UARTprintf("Pumping Clean... \n");)

									if(PRIME_POUCH_TUBES && (Cal_Number == 1 || PrimePouchTubes == 1))
									{
										DEBUG_PRINT(UARTprintf("Big Prime... \n");)
										RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
										FindPossitionZeroPump();
										PumpVolume(FW, PumpVol_air_bubble, 6000, 1);
										userDelay(valve_delay_after_air, 1);

										RunValveToPossition_Bidirectional_AbortReady(V_CLEAN, VALVE_STEPS_PER_POSITION);
										PumpVolume(FW, 240, 6000, 1);
										userDelay(valve_delay, 1);

										RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
										PumpVolume(FW, PumpVol_air_bubble + PumpVol_Large_air_bubble, 6000, 1);
										userDelay(valve_delay_after_air, 1);
									}

									RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
									FindPossitionZeroPump();
									uint16_t PumpSpeed = 3000;
									for (i = 0; i < Number_of_bubbles_Cals; i++) // Loop over air/solution cycle 3 times for single solution
									{
										RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
										if(i == 2)
											PumpSpeed = 6000;
										if(i == (Number_of_bubbles_Cals - 1))	// If this is the last bubble pump large bubble
											PumpVolume(FW, PumpVol_air_bubble + PumpVol_Large_air_bubble, PumpSpeed, 1);
										else
											PumpVolume(FW, PumpVol_air_bubble, PumpSpeed, 1);
										userDelay(valve_delay_after_air, 1);
										RunValveToPossition_Bidirectional_AbortReady(V_CLEAN, VALVE_STEPS_PER_POSITION);
										if(i == 0 && BUBBLES_IN_TUBE)	// First time through loop, clear bubble from tube
											PumpVolume(FW, PumpVol_tube_prime + PumpVol_Solution, PumpSpeed, 1);
										else if(i != (Number_of_bubbles_Cals - 1))	// If this is not the last plug only pump a small solution plug
											PumpVolume(FW, PumpVol_Solution, PumpSpeed, 1);
										else	// If this is the last plug, pump the whole plug
											PumpVolume(FW, PumpVol_Solution + PumpVol_Solution_plug, PumpSpeed, 1);

										userDelay(valve_delay, 1);
									}
									RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
									PumpVolume(FW, PumpVol_air_plug, PumpSpeed, 1);
		#ifdef KEEP_VALVE_AWAKE
									SleepValve();
		#endif
								}

								CollectISEmV(ISE_mV_Clean, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);

								T_Clean = MeasureTemperature(1);

#ifdef READ_REF_DURING_COND
								// Measure conductivity
								// Set RE and CE floating and close RE/CE loop for conductivity
								IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 0);
								IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
#else
								// Measure conductivity
								// Set RE and CE floating and close RE/CE loop for conductivity
								IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
								IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
#endif


								if((gui32Error & ABORT_ERRORS) == 0 /*&& Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean*/)
								{
									ConnectMemory(0);

									uint8_t Check = 0, attempt = 0;

#ifdef MEASURE_LOW_COND_RANGES
									// Set low current range
									// 10.7 uApp R = 309k + 499k = 808k
									IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
									IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);

									Check = 0;
									attempt = 0;
									while(Check != 1)
									{
										WaveGenSet(1);

										Check = CheckCond();
										if(attempt == 5)
										{
											gui32Error |= WAVE_GEN_FAIL;
											break;
										}

										if(Check != 1)
										{
											InitWaveGen(1);
											attempt++;
										}
									}

									Clean_Low_Current = ConductivityMovingAvg();

									WaveGenSet(0);	// Turn off waveform generator when switching ranges
#endif

									// Set mid current range
									// 20 uApp R = 430k
									IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 1);
									IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

									Check = 0;
									attempt = 0;
									while(Check != 1)
									{
										WaveGenSet(1);

										Check = CheckCond(COND_FREQ);
										if(attempt == 5)
										{
											gui32Error |= WAVE_GEN_FAIL;
											break;
										}

										if(Check != 1)
										{
											InitWaveGen(1, COND_FREQ);

											// Set mid current range
											// 20 uApp R = 430k
											IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 1);
											IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

											attempt++;
										}
									}


									float Clean_Cond_Mid_Raw = ConductivityMovingAvg(COND_FREQ);

									WaveGenSet(0);	// Turn off waveform generator when switching ranges

									// Set high current range
									// 45 uApp R = 180k
									IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
									IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

									//					userDelay(Cond_delay, 1);
//									Check = 0;
//									while(Check != 1)
//									{
//										WaveGenSet(1);
//
//										Check = CheckCond();
//										if(Check != 1)
//										{
//											InitWaveGen(1);
////											break;
//										}
//									}

									Check = 0;
									attempt = 0;
									while(Check != 1)
									{
										WaveGenSet(1);

										Check = CheckCond(COND_FREQ);
										if(attempt == 5)
										{
											gui32Error |= WAVE_GEN_FAIL;
											break;
										}

										if(Check != 1)
										{
											InitWaveGen(1, COND_FREQ);

											// Set high current range
											// 45 uApp R = 180k
											IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
											IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

											attempt++;
										}
									}

									float Clean_Cond_High_Raw = ConductivityMovingAvg(COND_FREQ);


									WaveGenSet(0);	// Turn off waveform generator

									ConnectMemory(1);


									// Current correct values before displaying
									float I_Mid, I_High;

									// Read the currents off the memory, these should be saved during the QC process
									EEPROMRead((uint32_t *) &I_Mid, OFFSET_COND_I_MID, 4);
									EEPROMRead((uint32_t *) &I_High, OFFSET_COND_I_HIGH, 4);

									if(I_Mid != I_Mid)
										I_Mid = 19.89 * 0.8;	// Average from circuits before ARV1_0B
									if(I_High != I_High)
										I_High = 43.57 * .812;	// Average from circuits before ARV1_0B

									Clean_Cond_Mid_Raw = (1000000 * I_Mid) / Clean_Cond_Mid_Raw;
									Clean_Cond_High_Raw = (1000000 * I_High) / Clean_Cond_High_Raw;

									MemoryWrite(Cal_page, OFFSET_CLEAN_MID_RAW, 4, (uint8_t *) &Clean_Cond_Mid_Raw);
									MemoryWrite(Cal_page, OFFSET_CLEAN_HIGH_RAW, 4, (uint8_t *) &Clean_Cond_High_Raw);

#ifdef CLEAN_COND
									DEBUG_PRINT(UARTprintf("Clean Mid: %d\n", (int) (Clean_Cond_Mid_Raw * 1000));)
									DEBUG_PRINT(UARTprintf("Clean High: %d\n", (int) (Clean_Cond_High_Raw * 1000));)
									ConnectMemory(0);

									CleanCond();

									// Set mid current range
									// 20 uApp R = 430k
									IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 1);
									IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

									//					userDelay(Cond_delay, 1);
//									uint8_t Check = 0;
//									while(Check != 1)
//									{
//										WaveGenSet(1);
//
//										Check = CheckCond();
//										if(Check != 1)
//										{
//											InitWaveGen(1);
////											break;
//										}
//									}

									Check = 0;
									attempt = 0;
									while(Check != 1)
									{
										WaveGenSet(1);

										Check = CheckCond();
										if(attempt == 5)
										{
											gui32Error |= WAVE_GEN_FAIL;
											break;
										}

										if(Check != 1)
										{
											InitWaveGen(1);
											attempt++;
										}
									}


									CalConductivityV2Mid = ConductivityMovingAvg();

									WaveGenSet(0);	// Turn off waveform generator when switching ranges

									// Set high current range
									// 45 uApp R = 180k
									IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
									IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

									//					userDelay(Cond_delay, 1);
//									Check = 0;
//									while(Check != 1)
//									{
//										WaveGenSet(1);
//
//										Check = CheckCond();
//										if(Check != 1)
//										{
//											InitWaveGen(1);
////											break;
//										}
//									}

									Check = 0;
									attempt = 0;
									while(Check != 1)
									{
										WaveGenSet(1);

										Check = CheckCond();
										if(attempt == 5)
										{
											gui32Error |= WAVE_GEN_FAIL;
											break;
										}

										if(Check != 1)
										{
											InitWaveGen(1);
											attempt++;
										}
									}

									CalConductivityV2High = ConductivityMovingAvg();


									WaveGenSet(0);	// Turn off waveform generator

									ConnectMemory(1);

									Clean_Cond_Mid_Raw = (1000000 * I_Mid) / CalConductivityV2Mid;
									Clean_Cond_High_Raw = (1000000 * I_High) / CalConductivityV2High;

									MemoryWrite(Cal_page, OFFSET_CLEAN_MID_RAW, 4, (uint8_t *) &Clean_Cond_Mid_Raw);
									MemoryWrite(Cal_page, OFFSET_CLEAN_HIGH_RAW, 4, (uint8_t *) &Clean_Cond_High_Raw);

									DEBUG_PRINT(UARTprintf("Clean Mid: %d\n", (int) (Clean_Cond_Mid_Raw * 1000));)
									DEBUG_PRINT(UARTprintf("Clean High: %d\n", (int) (Clean_Cond_High_Raw * 1000));)
#endif
								}

								if(REF_DRIFT != 0 && SATURATED_KCL_REF != 0)
									Ref_drift = Calculate_Ref_Drift(SATURATED_KCL_REF, T_Clean);
//								if(CLEAN_10X_CAL)
//								{
//									for(i = 0; i < 5; i++)
//									{
//										CleanAmperometrics_CurrentLimited(Ref_drift, Cal_Number, 0, 0);
//										CleanAmperometrics_CurrentLimited(Ref_drift, Cal_Number, 0, 1);
//									}
//
//								}
//								else

								gui32Error &= ~(CL_CLEANING_OUT_OF_RANGE);	// Clear the Cl cleaning failed flag in case we are repumping clean
#ifndef CV_CLEANING
#ifdef CC_CURRENT_LIMITED
								CleanAmperometrics_CurrentLimited(Ref_drift, Cal_Number, 0, OXIDE_REBUILD_TYPE);
#else
								CleanAmperometrics(Ref_drift, Cal_Number, 0, OXIDE_REBUILD_TYPE);
#endif	// CC_CURRENT_LIMITED
#else
								RunCVCleaningCycle(Ref_drift, Cal_Number, 0);
#endif	// CV_CLEANING

								ConnectMemory(1);
								MemoryWrite(Cal_page, OFFSET_CR_CLEAN_MV, 40, (uint8_t *) ISE_mV_Clean);

								update_Cal(Cal_Number);

								// Push air back into Clean port before moving to next solution
								if(BUBBLES_IN_TUBE)
								{
									RunValveToPossition_Bidirectional_AbortReady(V_CLEAN, VALVE_STEPS_PER_POSITION);
									//							PumpStepperRunStepSpeed_AbortReady(BW, Steps_tube_bubble, Speed_ISE);
									//							userDelay(valve_delay_after_air, 1);
									PumpVolume(BW, PumpVol_tube_bubble /*+ 13.77*/, Speed_Slow, 1);
//									PumpVolume(FW, 13.77, Speed_Slow, 1);
									userDelay(valve_delay, 1);
								}
							}
					}

#ifndef VALVE_STRUCT	// Breaks are required to prevent running into the next soltu9ion if using case/switch, but not if using if/else if
					break;
#endif
				}	// case V_CLEAN
#ifdef VALVE_STRUCT
				else if(Cal_Order[solution] == V_CAL_1)
#else
				case V_CAL_1:
#endif
				{
					//
					// Flow Chart Purple section, Cal 1
					//
					PrintTime();
					if((gui32Error & ABORT_ERRORS) == 0)
					{
//						update_Status(STATUS_CALIBRATION, OPERTAION_CAL_1);
//						update_Status(STATUS_CALIBRATION, solution + 2);
#ifdef PRINT_UART
						if(Sols-> Ca_EEP_Cal_1 == 0)
							DEBUG_PRINT(UARTprintf("Pumping Cal 6...\n");)
						else if(Sols->pH_EEP_Cal_2 < 9 && Sols->Ca_EEP_Cal_1 != 0)	// This is Cal 3/Cal 4 setup
							DEBUG_PRINT(UARTprintf("Pumping Cal 4...\n");)
						else
							DEBUG_PRINT(UARTprintf("Pumping Cal 1...\n");)
#endif

						if(PRIME_POUCH_TUBES && (Cal_Number == 1 || PrimePouchTubes == 1))
						{
							DEBUG_PRINT(UARTprintf("Big Prime... \n");)
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
							FindPossitionZeroPump();
							PumpVolume(FW, PumpVol_air_bubble, 6000, 1);
							userDelay(valve_delay_after_air, 1);

							RunValveToPossition_Bidirectional_AbortReady(V_CAL_1, VALVE_STEPS_PER_POSITION);
							PumpVolume(FW, 240, 6000, 1);
							userDelay(valve_delay, 1);

							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
							PumpVolume(FW, PumpVol_air_bubble + PumpVol_Large_air_bubble, 6000, 1);
							userDelay(valve_delay_after_air, 1);
						}

						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
						FindPossitionZeroPump();	// Find zero position of pump at beginning of every pump cycle
						uint16_t PumpSpeed = 3000;
						for (i = 0; i < Number_of_bubbles_Cals; i++) // Loop over air/solution cycle 3 times for single solution
						{
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
							if(i == 2)
								PumpSpeed = 6000;
							if(i == (Number_of_bubbles_Cals - 1))	// If this is the last bubble pump large bubble
								PumpVolume(FW, PumpVol_air_bubble + PumpVol_Large_air_bubble, PumpSpeed, 1);
							else
								PumpVolume(FW, PumpVol_air_bubble, PumpSpeed, 1);
							userDelay(valve_delay_after_air, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_CAL_1, VALVE_STEPS_PER_POSITION);
							if(i == 0 && BUBBLES_IN_TUBE)	// First time through loop, clear bubble from tube
								PumpVolume(FW, PumpVol_tube_prime + PumpVol_Solution, PumpSpeed, 1);
							else if(i != (Number_of_bubbles_Cals - 1))	// If this is not the last plug only pump a small solution plug
								PumpVolume(FW, PumpVol_Solution, PumpSpeed, 1);
							else	// If this is the last plug, pump the whole plug
								PumpVolume(FW, PumpVol_Solution + PumpVol_Solution_plug, PumpSpeed, 1);

							userDelay(valve_delay, 1);
						}
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						PumpVolume(FW, PumpVol_air_plug, PumpSpeed, 1);
		#ifdef KEEP_VALVE_AWAKE
						SleepValve();
		#endif
					}

					CollectISEmV(ISE_mV_Cal_1, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);

					// Measure temperature for 3 seconds
					T_Cal_1 = MeasureTemperature(1);

#ifdef READ_REF_DURING_COND
					// Measure conductivity
					// Set RE and CE floating and close RE/CE loop for conductivity
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 0);
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
#else
					// Measure conductivity
					// Set RE and CE floating and close RE/CE loop for conductivity
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
#endif

		//			float CalConductivityV3High = 0;
					if((gui32Error & ABORT_ERRORS) == 0)
					{
						if(Sols->pH_EEP_Cal_2 < 9.2)
						{
							ConnectMemory(0);

							if(gABoard >= ARV1_0B)
							{
								InitWaveGen(0, 1000);	// Change frequency to 1kHz

								// Set low current range
								// 10.7 uApp R = 309k + 499k = 808k
								IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
								IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);

								uint8_t Check = 0, attempt = 0;
								while(Check != 1)
								{
									WaveGenSet(1);

									Check = CheckCond(1000);
									if(attempt == 5)
									{
										gui32Error |= WAVE_GEN_FAIL;
										break;
									}

									if(Check != 1)
									{
										InitWaveGen(1, 1000);
										attempt++;
									}
								}

								CalConductivityV2Low_1k = ConductivityMovingAvg(1000);

								WaveGenSet(0);	// Turn off waveform generator when switching ranges

								InitWaveGen(0, 5000);	// Change frequency back to 5kHz
							}


							// Set low current range
							// 10.7 uApp R = 309k + 499k = 808k
							IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
							IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);

							uint8_t Check = 0, attempt = 0;
							while(Check != 1)
							{
								WaveGenSet(1);

								Check = CheckCond(COND_FREQ);
								if(attempt == 5)
								{
									gui32Error |= WAVE_GEN_FAIL;
									break;
								}

								if(Check != 1)
								{
									InitWaveGen(1, COND_FREQ);
									attempt++;
								}
							}

							CalConductivityV2Low = ConductivityMovingAvg(COND_FREQ);

							WaveGenSet(0);	// Turn off waveform generator when switching ranges

							// Set mid current range
							// 20 uApp R = 430k
							IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 1);
							IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

							//					userDelay(Cond_delay, 1);
//							Check = 0;
//							while(Check != 1)
//							{
//								WaveGenSet(1);
//
//								Check = CheckCond();
//								if(Check != 1)
//								{
//									InitWaveGen(1);
////									break;
//								}
//							}

							Check = 0;
							attempt = 0;
							while(Check != 1)
							{
								WaveGenSet(1);

								Check = CheckCond(COND_FREQ);
								if(attempt == 5)
								{
									gui32Error |= WAVE_GEN_FAIL;
									break;
								}

								if(Check != 1)
								{
									InitWaveGen(1, COND_FREQ);

									// Set mid current range
									// 20 uApp R = 430k
									IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 1);
									IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

									attempt++;
								}
							}

							CalConductivityV1Mid = ConductivityMovingAvg(COND_FREQ);

							WaveGenSet(0);	// Turn off waveform generator
						}
						else
						{
							ConnectMemory(0);

							// Set high current range
							// 45 uApp R = 180k
							IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
							IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

							//					userDelay(Cond_delay, 1);
//							uint8_t Check = 0;
//							while(Check != 1)
//							{
//								WaveGenSet(1);
//
//								Check = CheckCond();
//								if(Check != 1)
//								{
//									InitWaveGen(1);
////									break;
//								}
//							}

							uint8_t Check = 0, attempt = 0;
							while(Check != 1)
							{
								WaveGenSet(1);

								Check = CheckCond(COND_FREQ);
								if(attempt == 5)
								{
									gui32Error |= WAVE_GEN_FAIL;
									break;
								}

								if(Check != 1)
								{
									InitWaveGen(1, COND_FREQ);

									// Set high current range
									// 45 uApp R = 180k
									IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
									IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

									attempt++;
								}
							}

							CalConductivityV3High = ConductivityMovingAvg(COND_FREQ);

							WaveGenSet(0);	// Turn off waveform generator
						}
					}

					ConnectMemory(1);
					MemoryWrite(Cal_page, OFFSET_CR_CAL_1_MV, 40, (uint8_t *) ISE_mV_Cal_1);

					// Push air back into Cal 1 port before moving to next solution
					if(BUBBLES_IN_TUBE)
					{
						RunValveToPossition_Bidirectional_AbortReady(V_CAL_1, VALVE_STEPS_PER_POSITION);
						//				PumpStepperRunStepSpeed_AbortReady(BW, Steps_tube_bubble, Speed_ISE);
						//				userDelay(valve_delay_after_air, 1);
						PumpVolume(BW, PumpVol_tube_bubble /*+ 13.77*/, Speed_Slow, 1);
//						PumpVolume(FW, 13.77, Speed_Slow, 1);
						userDelay(valve_delay, 1);
					}

#ifdef H2_CALIBRATE_CAL6_B2_MIX	// TODO: Cal 6 + B2 mixing
					if(1)
					{
						// Prime B2
						DEBUG_PRINT(UARTprintf("Priming B2... \n");)
						RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);
						PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_Slow, 1);
						userDelay(valve_delay, 1);

						// 1/12/2022: Found our Alkalinity is consistently about 20 ppm high, believe this is caused by first 30 steps of acid pumping
						//	not moving solution but building pressure to move solution, subtracting this amount from calculations to get alkalinity
						float PumpVol_Buffer = (PumpVol_Solution + PumpVol_Solution_plug)/10 + (16.8 * 30.0/610.0);	// Calculate 1/10 the volume of the cal plug add on the dead volume pumping found during T1 mixing
						// Pump Cal 6 plug backwards
						// Pump mixed conditioner/solution back and mix with buffer
						DEBUG_PRINT(UARTprintf("Mixing %d uL of B2... \n", (int) PumpVol_Buffer);)

						float PumpVol_back = PumpVol_tube_prime_buffers + (PumpVol_air_plug - PumpVol_tube_bubble) + PumpVol_Solution + PumpVol_Solution_plug;

						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
						PumpVolume(BW, PumpVol_back, Speed_Slow, 1);
						userDelay(valve_delay, 1);
						int32_t Steps_to_align_pump = 1000 - (g_PumpStepsTravelled % 1000);
						if(Steps_to_align_pump == 1000)
							Steps_to_align_pump = 0;

						// Initialize floats to hold pump variables
						float PumpVolRev, Pump_Ratio;

						// Read from Tiva EEPROM the pump specs
						EEPROMRead((uint32_t *) &PumpVolRev, OFFSET_PUMP_VOL_PER_REV, 4);
						EEPROMRead((uint32_t *) &Pump_Ratio, OFFSET_PUMP_DEAD_SPOT, 4);

						if(PumpVolRev != PumpVolRev)
							PumpVolRev = 16.8;
						if(Pump_Ratio != Pump_Ratio)
							Pump_Ratio = 0.61;

						float Volume_to_align_pump = Steps_to_align_pump * PumpVolRev / Pump_Ratio / 1000;

						// Need to remove the dead spot volume, luckily with the PumpVolume code we will never stop the pump in the dead spot so it'll be an all or nothing calculation
						if(Steps_to_align_pump > 250)	// Check if we will be passing through dead spot, since we can't be inside the dead spot can merely check against center of dead spot
							Volume_to_align_pump -= (1 - Pump_Ratio) * PumpVolRev / Pump_Ratio;	// Remove the volume of the dead spot
						PumpStepperRunStepSpeed_AbortReady(FW, 1000 + Steps_to_align_pump, Speed_Slow);	// Buffer seemed to meter better if the pump was last pumping forward
						userDelay(500, 1);

						RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
						PumpVolume(FW, PumpVol_Buffer, Speed_Slow, 1);
						userDelay(500, 1);

						float PumpVol_forward = PumpVol_Solution + PumpVol_Solution_plug - PumpVolRev + PumpVol_air_bubble;		// Change this number to change where in mixing chamber the sample and buffer are mixed, pump forward the backward amount less a pump revolution and B2 volume

						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
						PumpVolume(FW, PumpVol_forward - Volume_to_align_pump, Speed_Slow, 1);
						userDelay(500, 1);

						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to sample

						PumpStepperMix(BW, 1500, Speed_ISE, 10);

						userDelay(5000, 1);	// Delay to let diffusion happen

						PumpVolume(FW, PumpVol_air_plug - PumpVol_air_bubble, Speed_ISE, 1);

						SleepValve();

						// Read ISEs
						float ISE_mV_Cal_1_B2[10];
						CollectISEmV(ISE_mV_Cal_1_B2, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);

						// Measure temperature for 3 seconds
						float T_Cal_1_B1 = MeasureTemperature(1);

						// Use Cr to estimate pH of mix
						// Should have the all the other points
						float pH_Cals[4] = {Calc_pH_TCor(Sols->pH_EEP_Cal_1, T_Cal_1, 25, 0, Sols->K_T_pH_Cal_1),
								Calc_pH_TCor(Sols->pH_EEP_Cal_2, T_Cal_2, 25, 0, Sols->K_T_pH_Cal_2),
								Calc_pH_TCor(Sols->pH_EEP_Rinse, T_Rinse, 25, 0, Sols->K_T_pH_Rinse),
								Calc_pH_TCor(Sols->pH_EEP_Clean, T_Clean, 25, Sols->K_T_pH_Clean_Sq, Sols->K_T_pH_Clean_Ln)};
						float mV_Cr[4];

						float max_r_squared = 0;
						float pH_Cr;
						for(i = 0; i < ISEs.pH_Cr.size; i++)
						{
							mV_Cr[0] = ISE_mV_Cal_1[ISEs.pH_Cr.index + i];
							mV_Cr[1] = ISE_mV_Cal_2[ISEs.pH_Cr.index + i];
							mV_Cr[2] = ISE_mV_Rinse[ISEs.pH_Cr.index + i];
							mV_Cr[3] = ISE_mV_Clean[ISEs.pH_Cr.index + i];

							float r_squared = RSQ(pH_Cals, mV_Cr, 4);
							float best_fit_slope = FindBestFitSlope(pH_Cals, mV_Cr, 4);
							float best_fit_int = (mV_Cr[0] + mV_Cr[1] + mV_Cr[2] + mV_Cr[3])/4 - best_fit_slope * (pH_Cals[0] + pH_Cals[1] + pH_Cals[2] + pH_Cals[3])/4;
							float pH_mix = (ISE_mV_Cal_1_B2[ISEs.pH_Cr.index + i] - best_fit_int)/best_fit_slope;
							DEBUG_PRINT(UARTprintf("pH Cr %d: %d/1000\n", i + 1, (int) (pH_mix * 1000));)
							if(r_squared > max_r_squared)
							{
								max_r_squared = r_squared;
								pH_Cr = pH_mix;
							}
						}

						// Calculate H2 slopes
						DEBUG_PRINT(UARTprintf("Using Cr sensor pH %d/1000\n", (int) (pH_Cr * 1000));)

						for(i = 0; i < ISEs.pH_H2.size; i++)
						{
							float H2_Slope = (ISE_mV_Cal_1_B2[ISEs.pH_H2.index + i] - ISE_mV_Cal_1[ISEs.pH_H2.index + i]) / (pH_Cr - pH_Cals[0]);
							DEBUG_PRINT(UARTprintf("H2 %d Slope: %d/1000\n", i + 1, (int) (H2_Slope * 1000));)
						}

						DEBUG_PRINT(UARTprintf("\nAssuming pH of 3.4\n");)

						for(i = 0; i < ISEs.pH_H2.size; i++)
						{
							float H2_Slope = (ISE_mV_Cal_1_B2[ISEs.pH_H2.index + i] - ISE_mV_Cal_1[ISEs.pH_H2.index + i]) / (3.4 - pH_Cals[0]);
							DEBUG_PRINT(UARTprintf("H2 %d Slope: %d/1000\n", i + 1, (int) (H2_Slope * 1000));)
						}

						// Push air back into Cal 1 port before moving to next solution
						if(BUBBLES_IN_TUBE)
						{
							RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);
							//				PumpStepperRunStepSpeed_AbortReady(BW, Steps_tube_bubble, Speed_ISE);
							//				userDelay(valve_delay_after_air, 1);
							PumpVolume(BW, PumpVol_tube_bubble /*+ 13.77*/, Speed_Slow, 1);
	//						PumpVolume(FW, 13.77, Speed_Slow, 1);
							userDelay(valve_delay, 1);
						}

						// Save the H2 mV
						MemoryWrite(Cal_page, OFFSET_H2_1_MV_CAL6_B2, 8, (uint8_t *) &ISE_mV_Cal_1_B2[ISEs.pH_H2.index]);	// Save both H2 mV to memory
					}
#endif

#ifndef VALVE_STRUCT	// Breaks are required to prevent running into the next soltu9ion if using case/switch, but not if using if/else if
					break;
#endif
				}	// case V_CAL_1
#ifdef VALVE_STRUCT
				else
#else
				default:
#endif
				{
					DEBUG_PRINT(UARTprintf("Unrecognized solution!\n");)
#ifndef VALVE_STRUCT	// Breaks are required to prevent running into the next soltu9ion if using case/switch, but not if using if/else if
					break;
#endif
				}
#ifndef VALVE_STRUCT
				}	// switch Cal_Order[solution]
#endif

				if(solution >= 3)	// After pumping last calibrant run the calculations
				{
					// Piecing together the calibration status that will be reported in the app
					// Bitwise, Calibrated, Alkalinity, pH, Calcium, Magnesium, Ammonium, Conductivity, Chlorine Check
					Calibration_Status = 0;
					uint8_t App_Status = 0;

//					// Create repump pointer variables so if a single solution needs to be repumped the data can be saved in the array that already exists for that solution
					if((gui32Error & ABORT_ERRORS) == 0)
					{
						// Determine which sensors passed calibration and which ones failed
						uint8_t ISE_Cal_Check[10] = {0,0,0,0,0,0,0,0,0,0};
						uint8_t *pH_H2_Cal_Check = &ISE_Cal_Check[ISEs.pH_H2.index];
						uint8_t *pH_Cr_Cal_Check = &ISE_Cal_Check[ISEs.pH_Cr.index];
						uint8_t *TH_Cal_Check = &ISE_Cal_Check[ISEs.TH.index];
						uint8_t *NH4_Cal_Check = &ISE_Cal_Check[ISEs.NH4.index];
						uint8_t *Ca_Cal_Check = &ISE_Cal_Check[ISEs.Ca.index];

						float ISE_Slope_CalT[10] = {0,0,0,0,0,0,0,0,0,0};
						float *pH_H2_Slope_CalT = &ISE_Slope_CalT[ISEs.pH_H2.index];
						float *pH_Cr_Slope_CalT = &ISE_Slope_CalT[ISEs.pH_Cr.index];
						float *TH_Slope_CalT = &ISE_Slope_CalT[ISEs.TH.index];
						float *NH4_Slope_CalT = &ISE_Slope_CalT[ISEs.NH4.index];
						float *Ca_Slope_CalT = &ISE_Slope_CalT[ISEs.Ca.index];

						// Define here so I can set to a sensor that passed calibration, doesn't matter which one just has to be one that passed, will redecide which one is best later based on linearity
						uint8_t L_pH_H2_Chosen = 0;
						uint8_t L_pH_Cr_Chosen = 0;
						uint8_t L_Ca_Chosen = 0;
						uint8_t L_TH_Chosen = 0;
						uint8_t L_NH4_Chosen = 0;

						float T_Cal = (T_Cal_1 + T_Cal_2 + T_Rinse) / 3;

						//
						// pH H2 Calibration
						//
						float pH_TCor_Rinse = Calc_pH_TCor(Sols->pH_EEP_Rinse, T_Cal, 25, 0, Sols->K_T_pH_Rinse);
						float pH_TCor_Cal_1 = Calc_pH_TCor(Sols->pH_EEP_Cal_1, T_Cal, 25, 0, Sols->K_T_pH_Cal_1);
						float pH_TCor_Cal_2 = Calc_pH_TCor(Sols->pH_EEP_Cal_2, T_Cal, 25, 0, Sols->K_T_pH_Cal_2);
						float pH_TCor_Clean = Calc_pH_TCor(Sols->pH_EEP_Clean, T_Cal, 25, Sols->K_T_pH_Clean_Sq, Sols->K_T_pH_Clean_Ln);

#ifdef LINEAR_PH_CORR
						float log_K_Ca_Mg = -5;
#else
						float log_K_Ca_Mg = LOG_K_CA_MG;
#endif

						// Values are concentrations, calculate rinse p-values
						float pCa_Rinse = Calc_pCa(Sols->Ca_EEP_Rinse, T_Cal, Sols->IS_RINSE);
						float pTH_Rinse = Calc_pTH(Sols->Ca_EEP_Rinse, Sols->TH_EEP_Rinse, log_K_Ca_Mg, T_Cal, Sols->IS_RINSE);
						float pNH4_Rinse = Calc_pNH4(Sols->NH4_EEP_Rinse, pH_TCor_Rinse, 0, T_Cal, Sols->IS_RINSE);

						if(ISEs.Config == PH_CL_CART)
							pH_TCor_Rinse = pH_TCor_Clean;

						for(i = 0; i < ISEs.pH_H2.size; i++)
						{
#ifdef CALIBRATE_H2_IN_CLEAN
							if(Sols->pH_EEP_Clean > 8.5 && Sols->pH_EEP_Cal_2 > 9)	// This is pH 9 Clean
								pH_H2_Slope_CalT[i] = (pH_H2_mV_Rinse[i] - pH_H2_mV_Cal_1[i])/(pH_TCor_Rinse - pH_TCor_Cal_1);
							else if(Sols->pH_EEP_Clean > 8.5 && Sols->pH_EEP_Cal_2 < 7)	// This is pH 9 Clean and Cal 5
								pH_H2_Slope_CalT[i] = (pH_H2_mV_Cal_2[i] - pH_H2_mV_Cal_1[i])/(pH_TCor_Cal_2 - pH_TCor_Cal_1);
							else
								pH_H2_Slope_CalT[i] = (pH_H2_mV_Clean[i] - pH_H2_mV_Cal_1[i])/(pH_TCor_Clean - pH_TCor_Cal_1);
#else
							pH_H2_Slope_CalT[i] = (pH_H2_mV_Rinse[i] - pH_H2_mV_Cal_1[i])/(pH_TCor_Rinse - pH_TCor_Cal_1);
#endif
							float pH_H2_Int = pH_H2_mV_Cal_1[i] - (pH_H2_Slope_CalT[i] * pH_TCor_Cal_1);
							MemoryWrite(Cal_page, OFFSET_ISE_1_INT + ((i + ISEs.pH_H2.index) * 4), 4, (uint8_t *) &pH_H2_Int);
							if(pH_H2_Slope_CalT[i] <= PH_SLOPE_HIGH && pH_H2_Slope_CalT[i] >= PH_SLOPE_LOW) // If slope is within range
							{
								pH_H2_Cal_Check[i] = 1;
								L_pH_H2_Chosen = i;
								App_Status |= (1 << APP_ALKALINITY);
							}
						}

						//
						// pH Cr Calibration
						//
						for(i = 0; i < ISEs.pH_Cr.size; i++)
						{
							float pH_Cr_Int;
							if(Sols->pH_EEP_Clean > 8.5 && Sols->pH_EEP_Cal_2 < 7)	// This is pH 9 Clean and Cal 5 setup
							{
								pH_Cr_Slope_CalT[i] = (pH_Cr_mV_Clean[i] - pH_Cr_mV_Cal_1[i])/(pH_TCor_Clean - pH_TCor_Cal_1);
								pH_Cr_Int = pH_Cr_mV_Cal_1[i] - (pH_Cr_Slope_CalT[i] * pH_TCor_Cal_1);
							}
							else
							{
								pH_Cr_Slope_CalT[i] = (pH_Cr_mV_Cal_2[i] - pH_Cr_mV_Cal_1[i])/(pH_TCor_Cal_2 - pH_TCor_Cal_1);
								pH_Cr_Int = pH_Cr_mV_Cal_1[i] - (pH_Cr_Slope_CalT[i] * pH_TCor_Cal_1);
							}

							MemoryWrite(Cal_page, OFFSET_ISE_1_INT + ((i + ISEs.pH_Cr.index) * 4), 4, (uint8_t *) &pH_Cr_Int);
							if(pH_Cr_Slope_CalT[i] <= PH_SLOPE_HIGH && pH_Cr_Slope_CalT[i] >= PH_SLOPE_LOW) // If slope is within range
							{
								pH_Cr_Cal_Check[i] = 1;
								L_pH_Cr_Chosen = i;
								App_Status |= (1 << APP_PH);
							}
						}

						//
						// Ca Calibration
						//

						// Values are concentration, calculate p-values
						float pCa_Cal_1, pCa_Cal_2, pCa_Clean;
						pCa_Cal_1 = Calc_pCa(Sols->Ca_EEP_Cal_1, T_Cal, Sols->IS_CAL_1);
						pCa_Cal_2 = Calc_pCa(Sols->Ca_EEP_Cal_2, T_Cal, Sols->IS_CAL_2);
						pCa_Clean = Calc_pCa(Sols->Ca_EEP_Clean, T_Cal, Sols->IS_CLEAN);

						for(i = 0; i < ISEs.Ca.size; i++)
						{
#ifdef LINEAR_PH_CORR
							float Ca_mpH, Ca_Int;
							if(Sols->Ca_EEP_Clean == 0 || Sols->Ca_EEP_Clean != Sols->Ca_EEP_Clean)
							{
								Ca_mpH = ((pCa_Rinse - pCa_Cal_1)*(Ca_mV_Rinse[i] - Ca_mV_Cal_2[i]) - (pCa_Rinse - pCa_Cal_2)*(Ca_mV_Rinse[i] - Ca_mV_Cal_1[i])) / ((pH_TCor_Rinse - pH_TCor_Cal_2)*(pCa_Rinse - pCa_Cal_1) - (pCa_Rinse - pCa_Cal_2)*(pH_TCor_Rinse - pH_TCor_Cal_1));
								Ca_Slope_CalT[i] = (-(pH_TCor_Rinse - pH_TCor_Cal_1)*(Ca_mV_Rinse[i] - Ca_mV_Cal_2[i]) + (pH_TCor_Rinse - pH_TCor_Cal_2)*(Ca_mV_Rinse[i] - Ca_mV_Cal_1[i])) / ((pH_TCor_Rinse - pH_TCor_Cal_2)*(pCa_Rinse - pCa_Cal_1) - (pCa_Rinse - pCa_Cal_2)*(pH_TCor_Rinse - pH_TCor_Cal_1));
								Ca_Int = Ca_mV_Cal_2[i] - (Ca_Slope_CalT[i] * pCa_Cal_2);
							}
							else	// If there is hardness in clean use that instead of Cal 1
							{
								Ca_mpH = ((pCa_Rinse - pCa_Clean)*(Ca_mV_Rinse[i] - Ca_mV_Cal_2[i]) - (pCa_Rinse - pCa_Cal_2)*(Ca_mV_Rinse[i] - Ca_mV_Clean[i])) / ((pH_TCor_Rinse - pH_TCor_Cal_2)*(pCa_Rinse - pCa_Clean) - (pCa_Rinse - pCa_Cal_2)*(pH_TCor_Rinse - pH_TCor_Clean));
								Ca_Slope_CalT[i] = (-(pH_TCor_Rinse - pH_TCor_Clean)*(Ca_mV_Rinse[i] - Ca_mV_Cal_2[i]) + (pH_TCor_Rinse - pH_TCor_Cal_2)*(Ca_mV_Rinse[i] - Ca_mV_Clean[i])) / ((pH_TCor_Rinse - pH_TCor_Cal_2)*(pCa_Rinse - pCa_Clean) - (pCa_Rinse - pCa_Cal_2)*(pH_TCor_Rinse - pH_TCor_Clean));
								Ca_Int = Ca_mV_Cal_2[i] - (Ca_Slope_CalT[i] * pCa_Cal_2);
							}

							if(Ca_mpH > 0)
							{
								DEBUG_PRINT(UARTprintf("Setting Ca pH Slope to 0!\n");)
								Ca_mpH = 0;
							}
							else if(Ca_mpH < -3)
							{
								DEBUG_PRINT(UARTprintf("Setting Ca pH Slope to -3!\n");)
								Ca_mpH = -3;
							}
							MemoryWrite(Cal_page, OFFSET_CA_1_LOG_K + (i * 4), 4, (uint8_t *) &Ca_mpH);
							MemoryWrite(Cal_page, OFFSET_ISE_1_INT + ((i + ISEs.Ca.index) * 4), 4, (uint8_t *) &Ca_Int);
#else	// LINEAR_PH_CORR
#ifndef CALIBRATE_CA_1_R
							if(Sols->pH_EEP_Cal_2 < 9  || Sols->Ca_EEP_Rinse > 140) // This is Cal 3 or hard rinse, not Cal 2
							{
								Ca_Slope_CalT[i] = (Ca_mV_Cal_2[i] - Ca_mV_Rinse[i]) / (pCa_Cal_2 - pCa_Rinse);	// Calibrated slope
								float Ca_Int = Ca_mV_Cal_2[i] - (Ca_Slope_CalT[i] * pCa_Cal_2);
								MemoryWrite(Cal_page, OFFSET_ISE_1_INT + ((i + ISEs.Ca.index) * 4), 4, (uint8_t *) &Ca_Int);

#ifdef PH_LOG_K
								if(Sols->pH_EEP_Cal_2 < 9 && Sols->Ca_EEP_Cal_1 != 0)	// This is Cal 3/Cal 4 setup
								{
									float Log_K_Ca_pH = log10(0.4833 * pow((Ca_mV_Cal_1[i] - Ca_mV_Rinse[i]), 2) + 11.603 * (Ca_mV_Cal_1[i] - Ca_mV_Rinse[i]) + 1.201);//1.6;	// Calculate a Log k
									MemoryWrite(Cal_page, OFFSET_CA_1_LOG_K + (i * 4), 4, (uint8_t *) &Log_K_Ca_pH);
								}
#endif	// PH_LOG_K
							}
							else
							{
								Ca_Slope_CalT[i] = (Ca_mV_Cal_2[i] - Ca_mV_Cal_1[i]) / (pCa_Cal_2 - pCa_Cal_1);	// Calibrated slope
								float Ca_Int = Ca_mV_Cal_1[i] - (Ca_Slope_CalT[i] * pCa_Cal_1);
								MemoryWrite(Cal_page, OFFSET_ISE_1_INT + ((i + ISEs.Ca.index) * 4), 4, (uint8_t *) &Ca_Int);
							}
							//					Ca_Int[i] = Ca_mV_Cal_1[i] - (Ca_Slope_CalT[i] * pCa_Cal_1);
#else
							Ca_Slope_CalT[i] = (Ca_mV_Rinse[i] - Ca_mV_Cal_1[i]) / (pCa_Rinse - pCa_Cal_1);	// Calibrated slope
							Ca_Int[i] = Ca_mV_Cal_1[i] - (Ca_Slope_CalT[i] * pCa_Cal_1);
							MemoryWrite(Cal_page, OFFSET_CA_1_MV_CAL_2, 4, (uint8_t *) &Ca_mV_Cal_2[0]);
							MemoryWrite(Cal_page, OFFSET_CA_2_MV_CAL_2, 4, (uint8_t *) &Ca_mV_Cal_2[1]);
#endif	// CALIBRATE_CA_1_R
#endif	// LINEAR_PH_CORR
							if(Ca_Slope_CalT[i] <= CA_SLOPE_HIGH && Ca_Slope_CalT[i] >= CA_SLOPE_LOW)
							{
								Ca_Cal_Check[i] = 1;
								L_Ca_Chosen = i;
								App_Status |= (1 << APP_CALCIUM);
							}
						}

						//
						// Magnesium and Total Hardness Calibration
						//
						float pTH_Cal_1, pTH_Cal_2, pTH_Clean;

						// Values are concentration, calculate p-values
						pTH_Cal_1 = Calc_pTH(Sols->Ca_EEP_Cal_1, Sols->TH_EEP_Cal_1, log_K_Ca_Mg, T_Cal, Sols->IS_CAL_1);
						pTH_Cal_2 = Calc_pTH(Sols->Ca_EEP_Cal_2, Sols->TH_EEP_Cal_2, log_K_Ca_Mg, T_Cal, Sols->IS_CAL_2);
						pTH_Clean = Calc_pTH(Sols->Ca_EEP_Clean, Sols->TH_EEP_Clean, log_K_Ca_Mg, T_Cal, Sols->IS_CLEAN);

						for(i = 0; i < ISEs.TH.size; i++)
						{
#ifndef CALIBRATE_TH_R_2
							TH_Slope_CalT[i] = (TH_mV_Cal_2[i] - TH_mV_Cal_1[i]) / (pTH_Cal_2 - pTH_Cal_1); // Calibrated slope
							TH_Int[i] = TH_mV_Cal_1[i] - (TH_Slope_CalT[i] * pTH_Cal_1);
#else

#ifdef LINEAR_PH_CORR
							float TH_mpH = -2, TH_Int;
							if(Sols->TH_EEP_Clean == 0 || Sols->TH_EEP_Clean != Sols->TH_EEP_Clean)
							{
								TH_Slope_CalT[i] = (TH_mV_Cal_2[i] - TH_mV_Rinse[i] - (TH_mpH * (pH_TCor_Cal_2 - pH_TCor_Rinse))) / (pTH_Cal_2 - pTH_Rinse); // Calibrated slope
								TH_Int = TH_mV_Cal_2[i] - (TH_Slope_CalT[i] * pTH_Cal_2);
							}
							else	// If there is hardness in Clean use that rather than Cal 1
							{
								TH_mpH = ((pTH_Rinse - pTH_Clean)*(TH_mV_Rinse[i] - TH_mV_Cal_2[i]) - (pTH_Rinse - pTH_Cal_2)*(TH_mV_Rinse[i] - TH_mV_Clean[i])) / ((pH_TCor_Rinse - pH_TCor_Cal_2)*(pTH_Rinse - pTH_Clean) - (pTH_Rinse - pTH_Cal_2)*(pH_TCor_Rinse - pH_TCor_Clean));
								TH_Slope_CalT[i] = (-(pH_TCor_Rinse - pH_TCor_Clean)*(TH_mV_Rinse[i] - TH_mV_Cal_2[i]) + (pH_TCor_Rinse - pH_TCor_Cal_2)*(TH_mV_Rinse[i] - TH_mV_Clean[i])) / ((pH_TCor_Rinse - pH_TCor_Cal_2)*(pTH_Rinse - pTH_Clean) - (pTH_Rinse - pTH_Cal_2)*(pH_TCor_Rinse - pH_TCor_Clean));
								TH_Int = TH_mV_Cal_2[i] - (TH_Slope_CalT[i] * pTH_Cal_2);
							}
							MemoryWrite(Cal_page, OFFSET_MG_1_PH_SLOPE + (i * 4), 4, (uint8_t *) &TH_mpH);
							MemoryWrite(Cal_page, OFFSET_ISE_1_INT + ((i + ISEs.TH.index) * 4), 4, (uint8_t *) &TH_Int);


#else	// LINEAR_PH_CORR
							TH_Slope_CalT[i] = (TH_mV_Cal_2[i] - TH_mV_Rinse[i]) / (pTH_Cal_2 - pTH_Rinse); // Calibrated slope
							float TH_Int = TH_mV_Cal_2[i] - (TH_Slope_CalT[i] * pTH_Cal_2);
							MemoryWrite(Cal_page, OFFSET_ISE_1_INT + ((i + ISEs.TH.index) * 4), 4, (uint8_t *) &TH_Int);

#endif	// LINEAR_PH_CORR

#ifdef PH_LOG_K
						if(Sols->pH_EEP_Cal_2 < 9 && Sols->TH_EEP_Cal_1 != 0)	// This is Cal 3/Cal 4 setup
						{
							float Log_K_TH_pH = log10(0.6456 * pow((TH_mV_Cal_1[i] - TH_mV_Rinse[i]), 2) + 24.499 * (TH_mV_Cal_1[i] - TH_mV_Rinse[i]) +1.1208); //2.1;	// TOD: Calculate a Log k
							MemoryWrite(Cal_page, OFFSET_TH_1_LOG_K + (i * 4), 4, (uint8_t *) &Log_K_TH_pH);
						}
#endif	// PH_LOG_K
#endif	// #ifndef CALIBRATE_TH_R_2
							if(TH_Slope_CalT[i] <= TH_SLOPE_HIGH && TH_Slope_CalT[i] >= TH_SLOPE_LOW)
							{
								TH_Cal_Check[i] = 1;
								L_TH_Chosen = i;
								App_Status |= (1 << APP_MAGNESIUM);
							}
						}

#ifdef TH_ITERATED_MATH
						// Calculate assuming Mg sensor (Nick's method)
						float pMg_Rinse, pMg_Cal_2;//, pMg_Cal_1;
						float Mg_Slope_CalT[2];
						if(ISEs.TH.size > 0)
						{
							pMg_Rinse = Calc_pMg(Sols->Ca_EEP_Rinse, Sols->TH_EEP_Rinse, T_Cal, Sols->IS_RINSE);
							//					pMg_Cal_1 = Calc_pMg(Ca_EEP_Cal_1, TH_EEP_Cal_1, T_Cal, IS_CAL_1);
							//					pMg_Cal_1 = Calc_pMg(Sols->Ca_EEP_Cal_1, Sols->TH_EEP_Cal_1, T_Cal, Sols->IS_CAL_1);
							pMg_Cal_2 = Calc_pMg(Sols->Ca_EEP_Cal_2, Sols->TH_EEP_Cal_2, T_Cal, Sols->IS_CAL_2);

							float /*Mg_Int[2],*/ log_K_Ca_Mg_Nick[2];
							for(i = 0; i < 2; i++)
							{
								Mg_Slope_CalT[i] = (TH_mV_Cal_2[i] - TH_mV_Rinse[i]) / (pMg_Cal_2 - pMg_Rinse); // Calibrated slope
								//						Mg_Int[i] = TH_mV_Rinse[i] - (Mg_Slope_CalT[i] * pMg_Rinse);
								//					if(Mg_Slope_CalT[i] <= TH_SLOPE_HIGH && Mg_Slope_CalT[i] >= TH_SLOPE_LOW)
								//					{
								//						TH_Cal_Check[i] = 1;
								//						L_TH_Chosen = i;
								//					}

								log_K_Ca_Mg_Nick[i] = -0.5;//0.0005 * pow((TH_mV_Cal_1[i] - TH_mV_Rinse[i]), 3) - 0.0158 * pow((TH_mV_Cal_1[i] - TH_mV_Rinse[i]), 2) + 0.2415 * (TH_mV_Cal_1[i] - TH_mV_Rinse[i]) - 1.5766;
							}


							for(i = 0; i < 2; i++)
							{
								MemoryWrite(Cal_page, OFFSET_MG_1_SLOPE + (i * 4), 4, (uint8_t *) &Mg_Slope_CalT[i]);
								//						MemoryWrite(Cal_page, OFFSET_MG_1_INT + (i * 4), 4, (uint8_t *) &Mg_Int[i]);
								MemoryWrite(Cal_page, OFFSET_MG_1_LOG_K + (i * 4), 4, (uint8_t *) &log_K_Ca_Mg_Nick[i]);
							}
						}
#endif

						//
						// NH4 Calibration
						//
						float pNH4_Cal_1, pNH4_Clean, pNH4_Cal_2;

						// Values are concentration, calculate p-values
						pNH4_Cal_1 = Calc_pNH4(Sols->NH4_EEP_Cal_1, pH_TCor_Cal_1, SM_NA_CAL_1, T_Cal, Sols->IS_CAL_1);
						pNH4_Clean = Calc_pNH4(Sols->NH4_EEP_Clean, pH_TCor_Clean, 0, T_Cal, Sols->IS_CLEAN);
						pNH4_Cal_2 = Calc_pNH4(Sols->NH4_EEP_Cal_2, pH_TCor_Cal_2, 0, T_Cal, Sols->IS_CAL_2);

						for(i = 0; i < ISEs.NH4.size; i++)
						{
							if(Sols->pH_EEP_Clean > 8.5)	// pH 9 Clean, calibrate between Cal 1 and Rinse
							{
								float NH4_Int;
								if(Sols->NH4_EEP_Cal_2 != 0)	// This is pH 9 Clean and Cal 5
								{
#ifdef NH4_PH_CORR
									float NH4_mpH = ((pNH4_Rinse - pNH4_Cal_1) * (NH4_mV_Rinse[i] - NH4_mV_Cal_2[i]) - (pNH4_Rinse - pNH4_Cal_2) * (NH4_mV_Rinse[i] - NH4_mV_Cal_1[i])) / ((pH_TCor_Rinse - pH_TCor_Cal_2) * (pNH4_Rinse - pNH4_Cal_1) - (pNH4_Rinse - pNH4_Cal_2) * (pH_TCor_Rinse - pH_TCor_Cal_1));
									NH4_Slope_CalT[i] = (-(pH_TCor_Rinse - pH_TCor_Cal_1) * (NH4_mV_Rinse[i] - NH4_mV_Cal_2[i]) + (pH_TCor_Rinse - pH_TCor_Cal_2) * (NH4_mV_Rinse[i] - NH4_mV_Cal_1[i])) / ((pH_TCor_Rinse - pH_TCor_Cal_2) * (pNH4_Rinse - pNH4_Cal_1) - (pNH4_Rinse - pNH4_Cal_2) * (pH_TCor_Rinse - pH_TCor_Cal_1));
									NH4_Int = NH4_mV_Cal_2[i] - (NH4_Slope_CalT[i] * pNH4_Cal_2);
									if(i < 2)	// Only have two spots specific to NH4 in memory currently...
										MemoryWrite(Cal_page, OFFSET_NH4_1_LOG_K + ((i) * 4), 4, (uint8_t *) &NH4_mpH);
									else if(i < 4)
										if(ISEs.Ca.size == 0)	// For disinfection cartridge I'm co-opting Ca interference memory locations
											MemoryWrite(Cal_page, OFFSET_CA_1_LOG_K + ((i - 2) * 4), 4, (uint8_t *) &NH4_mpH);
#else
									NH4_Slope_CalT[i] = (NH4_mV_Rinse[i] - NH4_mV_Cal_2[i]) / (pNH4_Rinse - pNH4_Cal_2);
									NH4_Int = NH4_mV_Cal_2[i] - (NH4_Slope_CalT[i] * pNH4_Cal_2);
#endif
								}
								else
								{
									NH4_Slope_CalT[i] = (NH4_mV_Rinse[i] - NH4_mV_Cal_1[i]) / (pNH4_Rinse - pNH4_Cal_1);
									NH4_Int = NH4_mV_Cal_1[i] - (NH4_Slope_CalT[i] * pNH4_Cal_1);
								}

								MemoryWrite(Cal_page, OFFSET_ISE_1_INT + ((i + ISEs.NH4.index) * 4), 4, (uint8_t *) &NH4_Int);
							}
							else if(Sols->pH_EEP_Cal_2 < 9 && Sols->NH4_EEP_Cal_1 == 0)	// This is Cal 3
							{
								NH4_Slope_CalT[i] = (NH4_mV_Clean[i] - NH4_mV_Cal_2[i]) / (pNH4_Clean - pNH4_Cal_2);
								float NH4_Int = NH4_mV_Cal_2[i] - (NH4_Slope_CalT[i] * pNH4_Cal_2);
								MemoryWrite(Cal_page, OFFSET_ISE_1_INT + ((i + ISEs.NH4.index) * 4), 4, (uint8_t *) &NH4_Int);

//#ifdef PH_LOG_K
//								if(Sols->pH_EEP_Cal_2 < 9 && Sols->NH4_EEP_Cal_1 != 0)	// This is Cal 3/Cal 4 setup
//								{
//									float Log_K_NH4_pH = -1;	// TOD: Calculate a Log k
//									MemoryWrite(Cal_page, OFFSET_NH4_1_LOG_K + (i * 4), 4, (uint8_t *) &Log_K_NH4_pH);
//								}
//#endif	// PH_LOG_K
							}
							else
							{
								NH4_Slope_CalT[i] = (NH4_mV_Clean[i] - NH4_mV_Cal_1[i]) / (pNH4_Clean - pNH4_Cal_1);
								float NH4_Int = NH4_mV_Cal_1[i] - (NH4_Slope_CalT[i] * pNH4_Cal_1);
								MemoryWrite(Cal_page, OFFSET_ISE_1_INT + ((i + ISEs.NH4.index) * 4), 4, (uint8_t *) &NH4_Int);
							}

							if(NH4_Slope_CalT[i] <= NH4_SLOPE_HIGH && NH4_Slope_CalT[i] >= NH4_SLOPE_LOW)
							{
								NH4_Cal_Check[i] = 1;
								L_NH4_Chosen = i;
								App_Status |= (1 << APP_AMMONIUM);
							}
						}

						//
						// Conductivity Calibration
						//
						// Adjust conductivity to use a fixed point at 61.8 of 270001.09 if factory cal wasn't read successfully
//						float CondLow;
//						float CalConductivityV1LowInv = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_COND_READ_LOW_POINT, 4));
//						if(CalConductivityV1LowInv == CalConductivityV1LowInv)
//						{
//							CondLow = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_COND_LOW_POINT_CAL, 4));
//							if(CalConductivityV1LowInv > 1)	// 8/9/2023: Factory Cal was done recording the mV difference, current adjust reading for new calibration
//								CalConductivityV1LowInv = (10.76 * 0.795) * 1000000 / CalConductivityV1LowInv;
//							else
//								CalConductivityV1LowInv *= 1000000;
//						}
//						else
//						{
//							CalConductivityV1LowInv = (10.76 * 0.795)  * 1000000 / (270001.09 * 2);
//							CondLow = 61.8;
//						}

						float I_Low, I_Mid, I_High;

						// Read the currents off the memory, these should be saved during the QC process
						EEPROMRead((uint32_t *) &I_Low, OFFSET_COND_I_LOW, 4);
						EEPROMRead((uint32_t *) &I_Mid, OFFSET_COND_I_MID, 4);
						EEPROMRead((uint32_t *) &I_High, OFFSET_COND_I_HIGH, 4);

						if(I_Low != I_Low)
							I_Low = 10.76 * 0.795;	// Average from circuits before ARV1_0B
						if(I_Mid != I_Mid)
							I_Mid = 19.89 * 0.8;	// Average from circuits before ARV1_0B
						if(I_High != I_High)
							I_High = 43.57 * .812;	// Average from circuits before ARV1_0B


						// Multiply the currents by 1,000,000 before dividing to avoid small numbers, in reality it's to convert our calibrants from uS to S but the computer math is better done this way
						float CalConductivityV2LowInv = (1000000 * I_Low) / CalConductivityV2Low;
						float CalConductivityV1MidInv = (1000000 * I_Mid) / CalConductivityV1Mid;
						float CalConductivityV2MidInv = (1000000 * I_Mid) / CalConductivityV2Mid;
						float CalConductivityV2HighInv = (1000000 * I_High) / CalConductivityV2High;
						float CalConductivityV3HighInv = (1000000 * I_High) / CalConductivityV3High;
		//				float CalConductivityDeltaVLow = CalConductivityV2LowInv - CalConductivityV1LowInv;
		//				float CalConductivityDeltaVMid = CalConductivityV2MidInv - CalConductivityV1MidInv;
		//				float CalConductivityDeltaVHigh = CalConductivityV3HighInv - CalConductivityV2HighInv;

#ifdef MEASURE_LOW_COND_RANGES
						Cal_5_Low_Current = (1000000 * I_Low) / Cal_5_Low_Current;
						Cal_5_Mid_Current = (1000000 * I_Mid) / Cal_5_Mid_Current;
						Rinse_Low_Current = (1000000 * I_Low) / Rinse_Low_Current;
						Clean_Low_Current = (1000000 * I_Low) / Clean_Low_Current;
#endif

						float CalConductivitySlopeLow, CalConductivitySlopeMid, CalConductivitySlopeHigh, CalConductivityKLow, CalConductivityKMid, CalConductivityKHigh;
						float CalConductivitySlopeLow_1k;
						if(1)	// This is Cal 3, Cal 3 is highest conductivity, Cal 1 is lowest conductivity
						{
							// Put the 3 calibrants used in the array twice each, order doesn't matter here because the array will be sorted from smallest to largest
							// Really only need 5 points with the highest conductivity calibrant in the array once, but to make it universal have an extra spot and the last spot will be ignored after sorting
//							float CalConds[3] = {Sols->Cond_EEP_Clean*(1 + Sols->Clean_Cond_TComp*(T_Cal - 25)), Sols->Cond_EEP_Cal_1*(1 + Sols->Cal_1_Cond_TComp*(T_Cal - 25)), Sols->Cond_EEP_Cal_2*(1 + Sols->Cal_2_Cond_TComp*(T_Cal - 25))};
							float CalConds[3] = {Sols->Cond_EEP_Rinse*(1 + Sols->Rinse_Cond_TComp*(T_Cal - 25)), Sols->Cond_EEP_Cal_1*(1 + Sols->Cal_1_Cond_TComp*(T_Cal - 25)), Sols->Cond_EEP_Cal_2*(1 + Sols->Cal_2_Cond_TComp*(T_Cal - 25))};
							SortArray(CalConds, 3);

//							CalConductivitySlopeLow =  (CalConductivityV2LowInv - CalConductivityV1LowInv) / (CalConds[0] - CondLow);
							CalConductivitySlopeLow =  (CalConductivityV2LowInv) / (CalConds[0]);
							CalConductivitySlopeMid = (CalConductivityV2MidInv - CalConductivityV1MidInv) / (CalConds[1] - CalConds[0]);
							CalConductivitySlopeHigh = (CalConductivityV3HighInv - CalConductivityV2HighInv) / (CalConds[2] - CalConds[1]);
							CalConductivityKLow = 0; //I_Low / CalConductivityV2Low - CalConductivitySlopeLow * CalConds[0]/1000000;
							CalConductivityKMid = I_Mid / CalConductivityV2Mid - CalConductivitySlopeMid * CalConds[1]/1000000;
							CalConductivityKHigh = I_High / CalConductivityV3High - CalConductivitySlopeHigh * CalConds[2]/1000000;

							if(gABoard >= ARV1_0B)
							{
								float I_Low_Alt;

								// Read the currents off the memory, these should be saved during the QC process
								EEPROMRead((uint32_t *) &I_Low_Alt, OFFSET_COND_ALT_I_LOW, 4);

								if(I_Low_Alt != I_Low_Alt)
								{
									float I_Low;
									EEPROMRead((uint32_t *) &I_Low, OFFSET_COND_I_LOW, 4);

									I_Low_Alt = I_Low * 1.22;	// Average from circuits before ARV1_0B
								}

								CalConductivityV2Low_1k = (1000000 * I_Low_Alt) / CalConductivityV2Low_1k;
								CalConductivitySlopeLow_1k = (CalConductivityV2Low_1k) / (CalConds[0]);
								DEBUG_PRINT(UARTprintf("1k Low Slope:\t%d/1000\n", (int) (CalConductivitySlopeLow_1k * 1000));)
								MemoryWrite(PAGE_CAL, OFFSET_CAL_COND_LOW_ALT_SLOPE, 4, (uint8_t *) &CalConductivitySlopeLow_1k);
							}
						}



						if(CalConductivitySlopeLow >= COND_SLOPE_1_LOW && CalConductivitySlopeLow <= COND_SLOPE_1_HIGH
								&& CalConductivitySlopeMid >= COND_SLOPE_2_LOW && CalConductivitySlopeMid <= COND_SLOPE_2_HIGH
								&& CalConductivitySlopeHigh >= COND_SLOPE_3_LOW && CalConductivitySlopeHigh <= COND_SLOPE_3_HIGH)
						{
							App_Status |= (1 << APP_COND);
						}

						// Mark if all sensors of a type failed calibration
						uint8_t Cal_failed = 0; // 0, 0, 0, 0, NH4, TH, Ca, pH

						// Clear any cal failed flags that have already been set in case we repumped a solution
						gui32Error &= ~(CAL_FAILED_COND_SLOPES | CAL_FAILED_PH_H2_SLOPES | CAL_FAILED_PH_CR_SLOPES | CAL_FAILED_TH_SLOPES | CAL_FAILED_NH4_SLOPES | CAL_FAILED_CA_SLOPES);

						// Determine if conductivity passed calibration, if it did not set error flag
						if(CalConductivitySlopeLow < COND_SLOPE_1_LOW || CalConductivitySlopeLow > COND_SLOPE_1_HIGH
								|| CalConductivitySlopeMid < COND_SLOPE_2_LOW || CalConductivitySlopeMid > COND_SLOPE_2_HIGH
								|| CalConductivitySlopeHigh < COND_SLOPE_3_LOW || CalConductivitySlopeHigh > COND_SLOPE_3_HIGH)
						{
							Cal_failed++;
							gui32Error |= CAL_FAILED_COND_SLOPES;
						}
						else
						{
							Calibration_Status |= 1 << 11;
							App_Status |= (1 << APP_COND);
						}


						if(ISEs.pH_H2.size > 0)
						{
							uint8_t pH_H2_passed = 0;
							for(i = 0; i < ISEs.pH_H2.size; i++)
								pH_H2_passed += pH_H2_Cal_Check[i];

							if(pH_H2_passed == 0)
							{
								Cal_failed++;
								gui32Error |= CAL_FAILED_PH_H2_SLOPES;
							}
						}

						if(ISEs.pH_Cr.size > 0)
						{
							uint8_t pH_Cr_passed = 0;
							for(i = 0; i < ISEs.pH_Cr.size; i++)
								pH_Cr_passed += pH_Cr_Cal_Check[i];

							if(pH_Cr_passed == 0)
							{
								Cal_failed++;
								gui32Error |= CAL_FAILED_PH_CR_SLOPES;
							}
						}

						if(ISEs.TH.size > 0)
						{
							uint8_t TH_passed = 0;
							for(i = 0; i < ISEs.TH.size; i++)
								TH_passed += TH_Cal_Check[i];

							if(TH_passed == 0)
							{
								Cal_failed++;
								gui32Error |= CAL_FAILED_TH_SLOPES;
							}
						}

						if(ISEs.NH4.size > 0)
						{
							uint8_t NH4_passed = 0;
							for(i = 0; i < ISEs.NH4.size; i++)
								NH4_passed += NH4_Cal_Check[i];

							if(NH4_passed == 0)
							{
								Cal_failed++;
								gui32Error |= CAL_FAILED_NH4_SLOPES;
							}
						}

						if(ISEs.Ca.size > 0)
						{
							uint8_t Ca_passed = 0;
							for(i = 0; i < ISEs.Ca.size; i++)
								Ca_passed += Ca_Cal_Check[i];

							if(Ca_passed == 0)
							{
								Cal_failed++;
								gui32Error |= CAL_FAILED_CA_SLOPES;
							}
						}

						// Choose sensors in calibration
						uint8_t L_Chosen_Sensors = 0;
						if(Sols->pH_EEP_Cal_2 < 9 && Sols->pH_EEP_Cal_2 > 7)	// This is Cal 3, can't calculate linearity because we only have 2 points
						{
							//
							// Choose sensors based on closest to theoretical slope
							//
							for(i = 0; i < ISEs.pH_H2.size; i++)
								if(abs_val((pH_H2_Slope_CalT[i] * (25 + 273)/(T_Cal + 273)) - (-59.9)) < abs_val((pH_H2_Slope_CalT[L_pH_H2_Chosen] * (25 + 273)/(T_Cal + 273)) - (-59.9)) && pH_H2_Cal_Check[i] == 1)
									L_pH_H2_Chosen = i;

							for(i = 0; i < ISEs.pH_Cr.size; i++)
								if(abs_val((pH_Cr_Slope_CalT[i] * (25 + 273)/(T_Cal + 273)) - (-59.9)) < abs_val((pH_Cr_Slope_CalT[L_pH_Cr_Chosen] * (25 + 273)/(T_Cal + 273)) - (-59.9)) && pH_Cr_Cal_Check[i] == 1)
									L_pH_Cr_Chosen = i;

							for(i = 0; i < ISEs.TH.size; i++)
								if(abs_val((TH_Slope_CalT[i] * (25 + 273)/(T_Cal + 273)) - (-29.5)) < abs_val((TH_Slope_CalT[L_TH_Chosen] * (25 + 273)/(T_Cal + 273)) - (-29.5)) && TH_Cal_Check[i] == 1)
									L_TH_Chosen = i;

							for(i = 0; i < ISEs.NH4.size; i++)
								if(abs_val((NH4_Slope_CalT[i] * (25 + 273)/(T_Cal + 273)) - (-59.9)) < abs_val((NH4_Slope_CalT[L_NH4_Chosen] * (25 + 273)/(T_Cal + 273)) - (-59.9)) && NH4_Cal_Check[i] == 1)
									L_NH4_Chosen = i;

							for(i = 0; i < ISEs.Ca.size; i++)
								if(abs_val((Ca_Slope_CalT[i] * (25 + 273)/(T_Cal + 273)) - (-29.5)) < abs_val((Ca_Slope_CalT[L_Ca_Chosen] * (25 + 273)/(T_Cal + 273)) - (-29.5)) && Ca_Cal_Check[i] == 1)
									L_Ca_Chosen = i;
						}
						else if(Sols->pH_EEP_Cal_2 < 7)	// This is Cal 5/pH 9 Clean/Cal 6 setup
						{
							//
							// Choose sensors based on closest to theoretical slope
							//
							for(i = 0; i < ISEs.pH_H2.size; i++)
								if(abs_val((pH_H2_Slope_CalT[i] * (25 + 273)/(T_Cal + 273)) - (-59.9)) < abs_val((pH_H2_Slope_CalT[L_pH_H2_Chosen] * (25 + 273)/(T_Cal + 273)) - (-59.9)) && pH_H2_Cal_Check[i] == 1)
									L_pH_H2_Chosen = i;

							//				float pH_r2[3], Ca_r2[2], TH_r2[2], NH4_r2[3];
							float ISE_r2[10] = {0,0,0,0,0,0,0,0,0,0};
							//				float *pH_H2_r2 = &ISE_r2[ISEs.pH_H2.index];
							float *pH_Cr_r2 = &ISE_r2[ISEs.pH_Cr.index];
							float *TH_r2 = &ISE_r2[ISEs.TH.index];
							float *NH4_r2 = &ISE_r2[ISEs.NH4.index];
							float *Ca_r2 = &ISE_r2[ISEs.Ca.index];

			//#ifdef TH_ITERATED_MATH
			//				float pMg_Rinse;
			//				pMg_Rinse = Calc_pMg(Ca_EEP_Rinse, TH_EEP_Rinse, T_Cal, IS_RINSE);
			//#endif

							// Calculate the R^2 value for each line to determine which is the most linear for each sensor type
							for(i = 0; i < ISEs.pH_Cr.size; i++)
							{
								float pH_xs[3] = {pH_TCor_Rinse, pH_TCor_Cal_1, pH_TCor_Clean};
								float pH_ys[3] = {pH_Cr_mV_Rinse[i], pH_Cr_mV_Cal_1[i], pH_Cr_mV_Clean[i]};
								pH_Cr_r2[i] = RSQ(pH_xs, pH_ys, 3);
							}
							for(i = 0; i < ISEs.Ca.size; i++)
							{
								float Ca_xs[3] = {pCa_Rinse, pCa_Clean, pCa_Cal_2};
								float Ca_ys[3] = {Ca_mV_Rinse[i], Ca_mV_Clean[i], Ca_mV_Cal_2[i]};
								Ca_r2[i] = RSQ(Ca_xs, Ca_ys, 3);
							}
							for(i = 0; i < ISEs.TH.size; i++)
							{
								float TH_xs[3] = {pTH_Rinse, pTH_Clean, pTH_Cal_2};
								float TH_ys[3] = {TH_mV_Rinse[i], TH_mV_Clean[i], TH_mV_Cal_2[i]};
								TH_r2[i] = RSQ(TH_xs, TH_ys, 3);
							}
							for(i = 0; i < ISEs.NH4.size; i++)
							{
								float NH4_xs[3] = {pNH4_Rinse, pNH4_Cal_1, pNH4_Cal_2};
								float NH4_ys[3] = {NH4_mV_Rinse[i], NH4_mV_Cal_1[i], NH4_mV_Cal_2[i]};
								NH4_r2[i] = RSQ(NH4_xs, NH4_ys, 3);
							}

							DEBUG_PRINT(UARTprintf("R Squared for each line using Prerinse:\n");)
							for(i = 0; i < ISEs.pH_Cr.size; i++)
								{DEBUG_PRINT(UARTprintf("pH %d: %d\n", i + 1, (int) (pH_Cr_r2[i] * 1000));)}
							for(i = 0; i < ISEs.TH.size; i++)
								{DEBUG_PRINT(UARTprintf("TH %d: %d\n", i + 1, (int) (TH_r2[i] * 1000));)}
							for(i = 0; i < ISEs.NH4.size; i++)
								{DEBUG_PRINT(UARTprintf("NH4 %d: %d\n", i + 1, (int) (NH4_r2[i] * 1000));)}
							for(i = 0; i < ISEs.Ca.size; i++)
								{DEBUG_PRINT(UARTprintf("Ca %d: %d\n", i + 1, (int) (Ca_r2[i] * 1000));)}

							//
							// Choose sensors based on linearity
							//
							for(i = 0; i < ISEs.pH_Cr.size; i++)
								if(pH_Cr_r2[i] > pH_Cr_r2[L_pH_Cr_Chosen] && pH_Cr_Cal_Check[i] == 1)
									L_pH_Cr_Chosen = i;

							for(i = 0; i < ISEs.TH.size; i++)
								if(TH_r2[i] > TH_r2[L_TH_Chosen] && TH_Cal_Check[i] == 1)
									L_TH_Chosen = i;

							for(i = 0; i < ISEs.NH4.size; i++)
								if(NH4_r2[i] > NH4_r2[L_NH4_Chosen] && NH4_Cal_Check[i] == 1)
									L_NH4_Chosen = i;

							for(i = 0; i < ISEs.Ca.size; i++)
								if(Ca_r2[i] > Ca_r2[L_Ca_Chosen] && Ca_Cal_Check[i] == 1)
									L_Ca_Chosen = i;
						}
						else
						{
							//				float pH_r2[3], Ca_r2[2], TH_r2[2], NH4_r2[3];
							float ISE_r2[10] = {0,0,0,0,0,0,0,0,0,0};
							//				float *pH_H2_r2 = &ISE_r2[ISEs.pH_H2.index];
							float *pH_Cr_r2 = &ISE_r2[ISEs.pH_Cr.index];
							float *TH_r2 = &ISE_r2[ISEs.TH.index];
							float *NH4_r2 = &ISE_r2[ISEs.NH4.index];
							float *Ca_r2 = &ISE_r2[ISEs.Ca.index];

			//#ifdef TH_ITERATED_MATH
			//				float pMg_Rinse;
			//				pMg_Rinse = Calc_pMg(Ca_EEP_Rinse, TH_EEP_Rinse, T_Cal, IS_RINSE);
			//#endif

							// Calculate the R^2 value for each line to determine which is the most linear for each sensor type
							for(i = 0; i < ISEs.pH_Cr.size; i++)
							{
								float pH_xs[3] = {pH_TCor_Rinse, pH_TCor_Cal_1, pH_TCor_Cal_2};
								float pH_ys[3] = {pH_Cr_mV_Rinse[i], pH_Cr_mV_Cal_1[i], pH_Cr_mV_Cal_2[i]};
								pH_Cr_r2[i] = RSQ(pH_xs, pH_ys, 3);
							}
							for(i = 0; i < ISEs.Ca.size; i++)
							{
								float Ca_xs[3] = {pCa_Rinse, pCa_Cal_1, pCa_Cal_2};
								float Ca_ys[3] = {Ca_mV_Rinse[i], Ca_mV_Cal_1[i], Ca_mV_Cal_2[i]};
								Ca_r2[i] = RSQ(Ca_xs, Ca_ys, 3);
							}
							for(i = 0; i < ISEs.TH.size; i++)
							{
								float TH_xs[3] = {pTH_Rinse, pTH_Cal_1, pTH_Cal_2};
								float TH_ys[3] = {TH_mV_Rinse[i], TH_mV_Cal_1[i], TH_mV_Cal_2[i]};
								TH_r2[i] = RSQ(TH_xs, TH_ys, 3);
							}
							for(i = 0; i < ISEs.NH4.size; i++)
							{
								float NH4_xs[3] = {pNH4_Rinse, pNH4_Cal_1, pNH4_Clean};
								float NH4_ys[3] = {NH4_mV_Rinse[i], NH4_mV_Cal_1[i], NH4_mV_Cal_2[i]};
								NH4_r2[i] = RSQ(NH4_xs, NH4_ys, 3);
							}

							DEBUG_PRINT(UARTprintf("R Squared for each line using Prerinse:\n");)
							for(i = 0; i < ISEs.pH_Cr.size; i++)
								{DEBUG_PRINT(UARTprintf("pH %d: %d\n", i + 1, (int) (pH_Cr_r2[i] * 1000));)}
							for(i = 0; i < ISEs.TH.size; i++)
								{DEBUG_PRINT(UARTprintf("TH %d: %d\n", i + 1, (int) (TH_r2[i] * 1000));)}
							for(i = 0; i < ISEs.NH4.size; i++)
								{DEBUG_PRINT(UARTprintf("NH4 %d: %d\n", i + 1, (int) (NH4_r2[i] * 1000));)}
							for(i = 0; i < ISEs.Ca.size; i++)
								{DEBUG_PRINT(UARTprintf("Ca %d: %d\n", i + 1, (int) (Ca_r2[i] * 1000));)}

							//
							// Choose sensors based on linearity
							//
							for(i = 0; i < ISEs.pH_Cr.size; i++)
								if(pH_Cr_r2[i] > pH_Cr_r2[L_pH_Cr_Chosen] && pH_Cr_Cal_Check[i] == 1)
									L_pH_Cr_Chosen = i;

							for(i = 0; i < ISEs.TH.size; i++)
								if(TH_r2[i] > TH_r2[L_TH_Chosen] && TH_Cal_Check[i] == 1)
									L_TH_Chosen = i;

							for(i = 0; i < ISEs.NH4.size; i++)
								if(NH4_r2[i] > NH4_r2[L_NH4_Chosen] && NH4_Cal_Check[i] == 1)
									L_NH4_Chosen = i;

							for(i = 0; i < ISEs.Ca.size; i++)
								if(Ca_r2[i] > Ca_r2[L_Ca_Chosen] && Ca_Cal_Check[i] == 1)
									L_Ca_Chosen = i;
						}

						L_Chosen_Sensors =
								(L_pH_H2_Chosen << ISEs.pH_H2.StorBit) |
								(L_pH_Cr_Chosen << ISEs.pH_Cr.StorBit) |
								(L_TH_Chosen << ISEs.TH.StorBit) |
								(L_NH4_Chosen << ISEs.NH4.StorBit) |
								(L_Ca_Chosen << ISEs.Ca.StorBit);

						if(Cal_failed == 0)
						{
							Calibration_Status |= 1;
							App_Status |= (1 >> APP_CALIBRATED);
						}

						for(i = 0; i < 10; i++)
							Calibration_Status |= (ISE_Cal_Check[i] << i + 1);

						// 8/12/2020: Removed rinse check from calibration status, added re-pump status so I can tell if re-pump occurred and what solution
						if(calibrants_to_pump == 5 && Cal_Order[4] == V_RINSE)
							Calibration_Status |= (0x00000001 << 19);
						else if(calibrants_to_pump == 5 && Cal_Order[4] == V_CAL_1)
							Calibration_Status |= (0x00000001 << 20);
						else if(calibrants_to_pump == 5 && Cal_Order[4] == V_CAL_2)
							Calibration_Status |= (0x00000001 << 21);
						else if(calibrants_to_pump == 5 && Cal_Order[4] == V_CLEAN)
							Calibration_Status |= (0x00000001 << 22);

						// Save this cal number if sensors passed calibration
						for(i = 0; i < 10; i++)
							if(ISE_Cal_Check[i])
								MemoryWrite(Cal_page, OFFSET_PH_1_LAST_P_CAL + i, 1, (uint8_t *) &Cal_Number);

						// Save cal data to cartridge memory
						MemoryWrite(Cal_page, OFFSET_T_CAL, 4, (uint8_t *) &T_Cal);
						for(i = 0; i < 10; i++)
						{
							MemoryWrite(Cal_page, OFFSET_ISE_1_SLOPE + (i * 4), 4, (uint8_t *) &ISE_Slope_CalT[i]);
		//					MemoryWrite(Cal_page, OFFSET_ISE_1_INT + (i * 4), 4, (uint8_t *) &ISE_Int[i]);
							MemoryWrite(Cal_page, OFFSET_CR_ISE_1_RINSE + (i * 4), 4, (uint8_t *) &ISE_mV_Rinse[i]);
//							MemoryWrite(Cal_page, OFFSET_CR_ISE_1_POST + (i * 4), 4, (uint8_t *) &ISE_mV_PostRinse[i]);
						}

						MemoryWrite(Cal_page, OFFSET_COND_R1_SLOPE, 4, (uint8_t *) &CalConductivitySlopeLow);
						MemoryWrite(Cal_page, OFFSET_COND_R1_INT, 4, (uint8_t *) &CalConductivityKLow);
						MemoryWrite(Cal_page, OFFSET_COND_R2_SLOPE, 4, (uint8_t *) &CalConductivitySlopeMid);
						MemoryWrite(Cal_page, OFFSET_COND_R2_INT, 4, (uint8_t *) &CalConductivityKMid);
						MemoryWrite(Cal_page, OFFSET_COND_R3_SLOPE, 4, (uint8_t *) &CalConductivitySlopeHigh);
						MemoryWrite(Cal_page, OFFSET_COND_R3_INT, 4, (uint8_t *) &CalConductivityKHigh);

						MemoryWrite(Cal_page, OFFSET_CAL_STATUS, 4, (uint8_t *) &Calibration_Status);
						MemoryWrite(Cal_page, OFFSET_CAL_DATA_ZERO, 1, &Zero);

						// Parse together the calibrated byte for the app reporting
						if((gui32Error & CL_CLEANING_OUT_OF_RANGE) == 0)
							App_Status |= (1 << APP_CHLORINE);
						MemoryWrite(Cal_page, OFFSET_CALIBRATED_STATUS, 1, &App_Status);

						// Calculate and save the chosen slope percentages
						if(ISEs.pH_H2.size > 0)
						{
							int16_t Slope_Percent = (float) ((pH_H2_Slope_CalT[L_pH_H2_Chosen] * (25.0 + 273.0)/(T_Cal + 273.0)) * 10000.0 / -59.9);	// Temperature correct slope to 25 then calculate it's percentage of theory
							MemoryWrite(Cal_page, OFFSET_ALK_SLOPE_PER, 2, (uint8_t *) &Slope_Percent);
						}
						else
						{
							uint16_t empty = 0xFFFF;
							MemoryWrite(Cal_page, OFFSET_ALK_SLOPE_PER, 2, (uint8_t *) &empty);
						}
						if(ISEs.pH_Cr.size > 0)
						{
							int16_t Slope_Percent = (float) ((pH_Cr_Slope_CalT[L_pH_Cr_Chosen] * (25.0 + 273.0)/(T_Cal + 273.0)) * 10000.0 / -59.9);	// Temperature correct slope to 25 then calculate it's percentage of theory
							MemoryWrite(Cal_page, OFFSET_PH_SLOPE_PER, 2, (uint8_t *) &Slope_Percent);
						}
						else
						{
							uint16_t empty = 0xFFFF;
							MemoryWrite(Cal_page, OFFSET_ALK_SLOPE_PER, 2, (uint8_t *) &empty);
						}
						if(ISEs.TH.size > 0)
						{
							int16_t Slope_Percent = (float) ((TH_Slope_CalT[L_TH_Chosen] * (25.0 + 273.0)/(T_Cal + 273.0)) * 10000.0 / -29.5);	// Temperature correct slope to 25 then calculate it's percentage of theory
							MemoryWrite(Cal_page, OFFSET_MG_SLOPE_PER, 2, (uint8_t *) &Slope_Percent);
						}
						else
						{
							uint16_t empty = 0xFFFF;
							MemoryWrite(Cal_page, OFFSET_ALK_SLOPE_PER, 2, (uint8_t *) &empty);
						}
						if(ISEs.NH4.size > 0)
						{
							int16_t Slope_Percent = (float) ((NH4_Slope_CalT[L_NH4_Chosen] * (25.0 + 273.0)/(T_Cal + 273.0)) * 10000.0 / -59.9);	// Temperature correct slope to 25 then calculate it's percentage of theory
							MemoryWrite(Cal_page, OFFSET_NH4_SLOPE_PER, 2, (uint8_t *) &Slope_Percent);
						}
						else
						{
							uint16_t empty = 0xFFFF;
							MemoryWrite(Cal_page, OFFSET_ALK_SLOPE_PER, 2, (uint8_t *) &empty);
						}
						if(ISEs.Ca.size > 0)
						{
							int16_t Slope_Percent = (float) ((Ca_Slope_CalT[L_Ca_Chosen] * (25.0 + 273.0)/(T_Cal + 273.0)) * 10000.0 / -29.5);	// Temperature correct slope to 25 then calculate it's percentage of theory
							MemoryWrite(Cal_page, OFFSET_CA_SLOPE_PER, 2, (uint8_t *) &Slope_Percent);
						}
						else
						{
							uint16_t empty = 0xFFFF;
							MemoryWrite(Cal_page, OFFSET_ALK_SLOPE_PER, 2, (uint8_t *) &empty);
						}

						// Calculate the conductivity slope percentage
						if(1)
						{
							// Put the 3 calibrants used in the array twice each, order doesn't matter here because the array will be sorted from smallest to largest
							// Really only need 5 points with the highest conductivity calibrant in the array once, but to make it universal have an extra spot and the last spot will be ignored after sorting
//							float CalConds[3] = {Sols->Cond_EEP_Clean*(1 + Sols->Clean_Cond_TComp*(T_Cal - 25)), Sols->Cond_EEP_Cal_1*(1 + Sols->Cal_1_Cond_TComp*(T_Cal - 25)), Sols->Cond_EEP_Cal_2*(1 + Sols->Cal_2_Cond_TComp*(T_Cal - 25))};
							float CalConds[3] = {Sols->Cond_EEP_Rinse*(1 + Sols->Rinse_Cond_TComp*(T_Cal - 25)), Sols->Cond_EEP_Cal_1*(1 + Sols->Cal_1_Cond_TComp*(T_Cal - 25)), Sols->Cond_EEP_Cal_2*(1 + Sols->Cal_2_Cond_TComp*(T_Cal - 25))};
							SortArray(CalConds, 3);

							float I_Low, I_Mid, I_High;

							// Read the currents off the memory, these should be saved during the QC process
							EEPROMRead((uint32_t *) &I_Low, OFFSET_COND_I_LOW, 4);
							EEPROMRead((uint32_t *) &I_Mid, OFFSET_COND_I_MID, 4);
							EEPROMRead((uint32_t *) &I_High, OFFSET_COND_I_HIGH, 4);

							if(I_Low != I_Low)
								I_Low = 10.76 * 0.795;	// Average from circuits before ARV1_0B
							if(I_Mid != I_Mid)
								I_Mid = 19.89 * 0.8;	// Average from circuits before ARV1_0B
							if(I_High != I_High)
								I_High = 43.57 * .812;	// Average from circuits before ARV1_0B

							float CondFactorySlope;
							if(gABoard >= ARV1_0B)
							{
								CondFactorySlope = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_FACTORY_COND_SLOPE_ALT, 4));

								if(CondFactorySlope == 0 || CondFactorySlope != CondFactorySlope)	// Check if there is valid data saved in the memory
									CondFactorySlope = 0.205;
							}
							else
							{
								CondFactorySlope = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_FACTORY_COND_SLOPE, 4));

								if(CondFactorySlope == 0 || CondFactorySlope != CondFactorySlope)	// Check if there is valid data saved in the memory
									CondFactorySlope = 0.18;
							}

							int16_t Slope_Percent = ((I_High * 1000000.0 / CalConductivityV3High - I_Mid * 1000000.0 / CalConductivityV2Mid) / (CalConds[2] - CalConds[1])) * 10000.0 / CondFactorySlope;	// Temperature correct slope to 25 then calculate it's percentage of theory

							MemoryWrite(Cal_page, OFFSET_COND_SLOPE_PER, 2, (uint8_t *) &Slope_Percent);
							DEBUG_PRINT(UARTprintf("Cond Slope Percentage: %d / 100\n", Slope_Percent);)
						}

						MemoryWrite(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1, &L_Chosen_Sensors);
						MemoryWrite(Cal_page, OFFSET_CAL_LOG_K, 4, (uint8_t *) &log_K_Ca_Mg);
						MemoryWrite(Cal_page, OFFSET_CR_ZERO, 1, &Zero);

//						MemoryWrite(Cal_page, OFFSET_CR_ISE_1_POST + (i * 4), 4, (uint8_t *) &ISE_mV_PostRinse[i]);
//						MemoryWrite(Cal_page, OFFSET_CR_T_POSTRINSE, 4, (uint8_t *) &T_PostRinse);

						// Save error data after all other data has been written to memory
						gui32Error &= ~(ROAM_RESET | APP_FILTER_ERROR);	// Turn off ROAM_RESET flag before saving error code
						MemoryWrite(Cal_page, OFFSET_CAL_ERROR, 4, (uint8_t *) &gui32Error);

						update_Cal(Cal_Number);

#ifdef PRINT_UART
						if(PRINT_RAW == 1)
						{
							// Print out raw calibration data to terminal
							DEBUG_PRINT(UARTprintf("Raw Calibration Data:\n");)
							DEBUG_PRINT(UARTprintf("Sensor\tRinse\tCal 1\tCal 2\tClean\n");)
							for(i = 0; i < ISEs.pH_H2.size; i++)
							{DEBUG_PRINT(UARTprintf("pH H2 %d\t%d\t%d\t%d\t%d\n", i + 1, (int) (pH_H2_mV_Rinse[i] * 1000), (int) (pH_H2_mV_Cal_1[i] * 1000), (int) (pH_H2_mV_Cal_2[i] * 1000), (int) (pH_H2_mV_Clean[i] * 1000));)}
							for(i = 0; i < ISEs.pH_Cr.size; i++)
							{DEBUG_PRINT(UARTprintf("pH %d\t%d\t%d\t%d\t%d\n", i + 1, (int) (pH_Cr_mV_Rinse[i] * 1000), (int) (pH_Cr_mV_Cal_1[i] * 1000), (int) (pH_Cr_mV_Cal_2[i] * 1000), (int) (ISE_mV_Clean[i + ISEs.pH_Cr.index] * 1000));)}
							for(i = 0; i < ISEs.TH.size; i++)
							{DEBUG_PRINT(UARTprintf("TH %d\t%d\t%d\t%d\t%d\n", i + 1, (int) (TH_mV_Rinse[i] * 1000), (int) (TH_mV_Cal_1[i] * 1000), (int) (TH_mV_Cal_2[i] * 1000), (int) (ISE_mV_Clean[i + ISEs.TH.index] * 1000));)}
							for(i = 0; i < ISEs.NH4.size; i++)
							{DEBUG_PRINT(UARTprintf("NH4 %d\t%d\t%d\t%d\t%d\n", i + 1, (int) (NH4_mV_Rinse[i] * 1000), (int) (NH4_mV_Cal_1[i] * 1000), (int) (NH4_mV_Cal_2[i] * 1000), (int) (NH4_mV_Clean[i] * 1000));)}
							for(i = 0; i < ISEs.Ca.size; i++)
							{DEBUG_PRINT(UARTprintf("Ca %d\t%d\t%d\t%d\t%d\n", i + 1, (int) (Ca_mV_Rinse[i] * 1000), (int) (Ca_mV_Cal_1[i] * 1000), (int) (Ca_mV_Cal_2[i] * 1000), (int) (ISE_mV_Clean[i + ISEs.Ca.index] * 1000));)}
							DEBUG_PRINT(UARTprintf("\n");)

							DEBUG_PRINT(UARTprintf("Raw Conductivity Data:\n");)
							DEBUG_PRINT(UARTprintf("Cond Low\t%d\n", (int) (CalConductivityV2Low * 1000));)
							DEBUG_PRINT(UARTprintf("Cond Mid\t%d\t%d\n", (int) (CalConductivityV1Mid * 1000), (int) (CalConductivityV2Mid * 1000));)
							DEBUG_PRINT(UARTprintf("Cond High\t%d\t%d\n\n", (int) (CalConductivityV2High * 1000), (int) (CalConductivityV3High * 1000));)

							DEBUG_PRINT(UARTprintf("Raw Conductivity Data:\n");)
							if(gABoard >= ARV1_0B)
							{
								DEBUG_PRINT(UARTprintf("Cond Low 1k\t0\t%d\n", (int) (CalConductivityV2Low_1k * 1000));)
							}
							DEBUG_PRINT(UARTprintf("Cond Low\t0\t%d\n", (int) (CalConductivityV2LowInv * 1000));)
							DEBUG_PRINT(UARTprintf("Cond Mid\t%d\t%d\n", (int) (CalConductivityV1MidInv * 1000), (int) (CalConductivityV2MidInv * 1000));)
							DEBUG_PRINT(UARTprintf("Cond High\t%d\t%d\n\n", (int) (CalConductivityV2HighInv * 1000), (int) (CalConductivityV3HighInv * 1000));)

#ifdef MEASURE_LOW_COND_RANGES
							DEBUG_PRINT(UARTprintf("Cal 6\t%d\t%d\n", (int) (CalConductivityV2LowInv * 1000), (int) (CalConductivityV1MidInv * 1000));)
							DEBUG_PRINT(UARTprintf("Rinse\t%d\t%d\t%d\n", (int) (Rinse_Low_Current * 1000), (int) (Rinse_Cond_Mid_Raw * 1000), (int) (Rinse_Cond_High_Raw * 1000));)
							DEBUG_PRINT(UARTprintf("Clean\t%d\t%d\t%d\n", (int) (Clean_Low_Current * 1000), (int) (CalConductivityV2MidInv * 1000), (int) (CalConductivityV2HighInv * 1000));)
							DEBUG_PRINT(UARTprintf("Cal 5\t%d\t%d\t%d\n", (int) (Cal_5_Low_Current * 1000), (int) (Cal_5_Mid_Current * 1000), (int) (CalConductivityV3HighInv * 1000));)
#endif


							DEBUG_PRINT(UARTprintf("Calculated p-values:\n");)
							DEBUG_PRINT(UARTprintf("\tRinse\tCal 1\tCal 2\tClean\n");)
							if(ISEs.pH_H2.size > 0 || ISEs.pH_Cr.size > 0)
							{DEBUG_PRINT(UARTprintf("pH\t%d\t%d\t%d\t%d\n", (int) (pH_TCor_Rinse * 1000), (int) (pH_TCor_Cal_1 * 1000), (int) (pH_TCor_Cal_2 * 1000), (int) (pH_TCor_Clean * 1000));)}
							if(ISEs.TH.size > 0)
							{DEBUG_PRINT(UARTprintf("pTH\t%d\t%d\t%d\t%d\n", (int) (pTH_Rinse * 1000), (int) (pTH_Cal_1 * 1000), (int) (pTH_Cal_2 * 1000), (int) (pTH_Clean * 1000));)}
							if(ISEs.NH4.size > 0)
							{DEBUG_PRINT(UARTprintf("pNH4\t%d\t%d\t%d\t%d\n", (int) (pNH4_Rinse * 1000), (int) (pNH4_Cal_1 * 1000), (int) (pNH4_Cal_2 * 1000), (int) (pNH4_Clean * 1000));)}
							if(ISEs.Ca.size > 0)
							{DEBUG_PRINT(UARTprintf("pCa\t%d\t%d\t%d\t%d\n", (int) (pCa_Rinse * 1000), (int) (pCa_Cal_1 * 1000), (int) (pCa_Cal_2 * 1000), (int) (pCa_Clean * 1000));)}
#ifdef TH_ITERATED_MATH
							if(ISEs.TH.size > 0)
							{DEBUG_PRINT(UARTprintf("pMg\t%d\t\t%d\n", (int) (pMg_Rinse * 1000), (int) (pMg_Cal_2 * 1000));)}
#endif

							DEBUG_PRINT(UARTprintf("Temperatures:\n");)
							DEBUG_PRINT(UARTprintf("Prerinse\t%d\n", (int) (T_Rinse * 1000));)
							DEBUG_PRINT(UARTprintf("Cal 1\t%d\n", (int) (T_Cal_1 * 1000));)
							DEBUG_PRINT(UARTprintf("Cal 2\t%d\n", (int) (T_Cal_2 * 1000));)
							if(ISEs.Config != PH_CL_CART)
							{DEBUG_PRINT(UARTprintf("Clean\t%d\n", (int) (T_Clean * 1000));)}
						}

						if(Sols->pH_EEP_Cal_2 < 9)
						{
							DEBUG_PRINT(UARTprintf("Chosen Sensors by slope:\n");)
							DEBUG_PRINT(UARTprintf("pH H2: %d\n", L_pH_H2_Chosen);)
							DEBUG_PRINT(UARTprintf("pH Cr: %d\n", L_pH_Cr_Chosen);)
							DEBUG_PRINT(UARTprintf("TH: %d\n", L_TH_Chosen);)
							DEBUG_PRINT(UARTprintf("NH4: %d\n", L_NH4_Chosen);)
							DEBUG_PRINT(UARTprintf("Ca: %d\n\n", L_Ca_Chosen);)
						}
						else
						{
							DEBUG_PRINT(UARTprintf("Linear Chosen Sensors:\n");)
							if(ISEs.pH_H2.size > 0)
							{DEBUG_PRINT(UARTprintf("pH H2 sensor doesn't have a third point to check linearity against, can't pick sensor!\n");)}
							DEBUG_PRINT(UARTprintf("pH Cr: %d\n", L_pH_Cr_Chosen);)
							DEBUG_PRINT(UARTprintf("TH: %d\n", L_TH_Chosen);)
							DEBUG_PRINT(UARTprintf("NH4: %d\n", L_NH4_Chosen);)
							DEBUG_PRINT(UARTprintf("Ca: %d\n\n", L_Ca_Chosen);)
						}

#ifdef PH_LOG_K
						// Print out calculated slopes to terminal
						DEBUG_PRINT(UARTprintf("Slopes:\n");)
						for(i = 0; i < ISEs.pH_H2.size; i++)
						{DEBUG_PRINT(UARTprintf("pH H2 %d\t%d\n", (i + 1), (int) (pH_H2_Slope_CalT[i] * 1000));)}
						for(i = 0; i < ISEs.pH_Cr.size; i++)
						{DEBUG_PRINT(UARTprintf("pH Cr %d\t%d\n", (i + 1), (int) (pH_Cr_Slope_CalT[i] * 1000));)}
						for(i = 0; i < ISEs.TH.size; i++)
						{DEBUG_PRINT(UARTprintf("TH %d\t%d\tLog K\t%d\n", (i + 1), (int) (TH_Slope_CalT[i] * 1000), (int) (Build_float(MemoryRead(Cal_page, OFFSET_TH_1_LOG_K + (i * 4), 4)) * 1000));)}

						for(i = 0; i < ISEs.NH4.size; i++)
						{DEBUG_PRINT(UARTprintf("NH4 %d\t%d\n", (i + 1), (int) (NH4_Slope_CalT[i] * 1000));)}
						for(i = 0; i < ISEs.Ca.size; i++)
						{DEBUG_PRINT(UARTprintf("Ca %d\t%d\tLog K\t%d\n", (i + 1), (int) (Ca_Slope_CalT[i] * 1000), (int) (Build_float(MemoryRead(Cal_page, OFFSET_CA_1_LOG_K + (i * 4), 4)) * 1000));)}
#ifdef TH_ITERATED_MATH
						if(ISEs.TH.size > 0)
						{
							for(i = 0; i < 2; i++)
							{
								DEBUG_PRINT(UARTprintf("Mg %d\t%d\n", (i + 1), (int) (Mg_Slope_CalT[i] * 1000));)
							}
						}
#endif	// TH_ITERATED_MATH
#else	// PH_LOG_K
						// Print out calculated slopes to terminal
						DEBUG_PRINT(UARTprintf("Slopes:\n");)
						for(i = 0; i < ISEs.pH_H2.size; i++)
						{DEBUG_PRINT(UARTprintf("pH H2 %d\t%d\n", (i + 1), (int) (pH_H2_Slope_CalT[i] * 1000));)}
						for(i = 0; i < ISEs.pH_Cr.size; i++)
						{DEBUG_PRINT(UARTprintf("pH Cr %d\t%d\n", (i + 1), (int) (pH_Cr_Slope_CalT[i] * 1000));)}
#ifdef LINEAR_PH_CORR
						for(i = 0; i < ISEs.TH.size; i++)
						{DEBUG_PRINT(UARTprintf("TH %d\t%d\t%d\n", (i + 1), (int) (TH_Slope_CalT[i] * 1000), (int) (Build_float(MemoryRead(Cal_page, OFFSET_MG_1_PH_SLOPE + (i * 4), 4)) * 1000));)}
#else	// LINEAR_PH_CORR
						for(i = 0; i < ISEs.TH.size; i++)
						{DEBUG_PRINT(UARTprintf("TH %d\t%d\t%d\n", (i + 1), (int) (TH_Slope_CalT[i] * 1000));)}
#endif	// LINEAR_PH_CORR

#ifdef NH4_PH_CORR
						for(i = 0; i < ISEs.NH4.size; i++)
						{DEBUG_PRINT(UARTprintf("NH4 %d\t%d\t%d\n", (i + 1), (int) (NH4_Slope_CalT[i] * 1000), (int) ( ((pNH4_Rinse - pNH4_Cal_1) * (NH4_mV_Rinse[i] - NH4_mV_Cal_2[i]) - (pNH4_Rinse - pNH4_Cal_2) * (NH4_mV_Rinse[i] - NH4_mV_Cal_1[i])) / ((pH_TCor_Rinse - pH_TCor_Cal_2) * (pNH4_Rinse - pNH4_Cal_1) - (pNH4_Rinse - pNH4_Cal_2) * (pH_TCor_Rinse - pH_TCor_Cal_1)) * 1000 ));)}
#else
						for(i = 0; i < ISEs.NH4.size; i++)
						{DEBUG_PRINT(UARTprintf("NH4 %d\t%d\n", (i + 1), (int) (NH4_Slope_CalT[i] * 1000));)}
#endif

#ifdef LINEAR_PH_CORR
						for(i = 0; i < ISEs.Ca.size; i++)
						{DEBUG_PRINT(UARTprintf("Ca %d\t%d\t%d\n", (i + 1), (int) (Ca_Slope_CalT[i] * 1000), (int) (Build_float(MemoryRead(Cal_page, OFFSET_CA_1_LOG_K + (i * 4), 4)) * 1000));)}
#else // LINEAR_PH_CORR
						for(i = 0; i < ISEs.Ca.size; i++)
						{DEBUG_PRINT(UARTprintf("Ca %d\t%d\n", (i + 1), (int) (Ca_Slope_CalT[i] * 1000));)}
#endif// LINEAR_PH_CORR
#ifdef TH_ITERATED_MATH
						if(ISEs.TH.size > 0)
						{
							for(i = 0; i < 2; i++)
							{
								DEBUG_PRINT(UARTprintf("Mg %d\t%d\n", (i + 1), (int) (Mg_Slope_CalT[i] * 1000));)
							}
						}
#endif	// TH_ITERATED_MATH
#endif	// PH_LOG_K

						if(gABoard >= ARV1_0B)
						{
							DEBUG_PRINT(UARTprintf("Cond Low 1kHz\t%d\n", (int) (CalConductivitySlopeLow_1k * 1000));)
						}
						DEBUG_PRINT(UARTprintf("Cond Low\t%d\t%d\n", (int) (CalConductivitySlopeLow * 1000), (int) (CalConductivityKLow * 1000));)
						DEBUG_PRINT(UARTprintf("Cond Mid\t%d\t%d\n", (int) (CalConductivitySlopeMid * 1000), (int) (CalConductivityKMid * 1000));)
						DEBUG_PRINT(UARTprintf("Cond High\t%d\t%d\n", (int) (CalConductivitySlopeHigh * 1000), (int) (CalConductivityKHigh * 1000));)

						DEBUG_PRINT(UARTprintf("\n");)

						// Print out chosen slopes to terminal
						DEBUG_PRINT(UARTprintf("Chosen slopes:\n");)
						if(ISEs.pH_H2.size > 0)
						{DEBUG_PRINT(UARTprintf("pH H2\t%d\n", (int) (pH_H2_Slope_CalT[L_pH_H2_Chosen] * 1000));)}
						if(ISEs.pH_Cr.size > 0)
						{DEBUG_PRINT(UARTprintf("pH\t%d\n", (int) (pH_Cr_Slope_CalT[L_pH_Cr_Chosen] * 1000));)}
#ifdef LINEAR_PH_CORR
						if(ISEs.TH.size > 0)
						{DEBUG_PRINT(UARTprintf("TH\t%d\t%d\n", (int) (TH_Slope_CalT[L_TH_Chosen] * 1000), (int) (Build_float(MemoryRead(Cal_page, OFFSET_MG_1_PH_SLOPE + (L_TH_Chosen * 4), 4)) * 1000));)}
#else
						if(ISEs.TH.size > 0)
						{DEBUG_PRINT(UARTprintf("TH\t%d\n", (int) (TH_Slope_CalT[L_TH_Chosen] * 1000));)}
#endif

#ifdef NH4_PH_CORR
						if(ISEs.NH4.size > 0)
						{DEBUG_PRINT(UARTprintf("NH4\t%d\t%d\n", (int) (NH4_Slope_CalT[L_NH4_Chosen] * 1000), (int) (((pNH4_Rinse - pNH4_Cal_1) * (NH4_mV_Rinse[L_NH4_Chosen] - NH4_mV_Cal_2[L_NH4_Chosen]) - (pNH4_Rinse - pNH4_Cal_2) * (NH4_mV_Rinse[L_NH4_Chosen] - NH4_mV_Cal_1[L_NH4_Chosen])) / ((pH_TCor_Rinse - pH_TCor_Cal_2) * (pNH4_Rinse - pNH4_Cal_1) - (pNH4_Rinse - pNH4_Cal_2) * (pH_TCor_Rinse - pH_TCor_Cal_1)) * 1000));)}
#else
						if(ISEs.NH4.size > 0)
						{DEBUG_PRINT(UARTprintf("NH4\t%d\n", (int) (NH4_Slope_CalT[L_NH4_Chosen] * 1000));)}
#endif

#ifdef LINEAR_PH_CORR
						if(ISEs.Ca.size > 0)
						{DEBUG_PRINT(UARTprintf("Ca\t%d\t%d\n", (int) (Ca_Slope_CalT[L_Ca_Chosen] * 1000), (int) (Build_float(MemoryRead(Cal_page, OFFSET_CA_1_LOG_K + (L_Ca_Chosen * 4), 4)) * 1000));)}
#else
						if(ISEs.Ca.size > 0)
						{DEBUG_PRINT(UARTprintf("Ca\t%d\n", (int) (Ca_Slope_CalT[L_Ca_Chosen] * 1000));)}
#endif

						DEBUG_PRINT(UARTprintf("\n");)

						DEBUG_PRINT(UARTprintf("Sensor calibration status: \n");)

						if(ISEs.pH_H2.size > 0)
						{
							DEBUG_PRINT(UARTprintf("pH H2:");)
									for(i = 0; i < ISEs.pH_H2.size; i++)
									{DEBUG_PRINT(UARTprintf("\t%d", pH_H2_Cal_Check[i]);)}
							DEBUG_PRINT(UARTprintf("\n");)
						}
						if(ISEs.pH_Cr.size > 0)
						{
							DEBUG_PRINT(UARTprintf("pH Cr:");)
									for(i = 0; i < ISEs.pH_Cr.size; i++)
									{DEBUG_PRINT(UARTprintf("\t%d", pH_Cr_Cal_Check[i]);)}
							DEBUG_PRINT(UARTprintf("\n");)
						}
						if(ISEs.TH.size > 0)
						{
							DEBUG_PRINT(UARTprintf("TH:");)
									for(i = 0; i < ISEs.TH.size; i++)
									{DEBUG_PRINT(UARTprintf("\t%d", TH_Cal_Check[i]);)}
							DEBUG_PRINT(UARTprintf("\n");)
						}
						if(ISEs.NH4.size > 0)
						{
							DEBUG_PRINT(UARTprintf("NH4:");)
									for(i = 0; i < ISEs.NH4.size; i++)
									{DEBUG_PRINT(UARTprintf("\t%d", NH4_Cal_Check[i]);)}
							DEBUG_PRINT(UARTprintf("\n");)
						}
						if(ISEs.Ca.size > 0)
						{
							DEBUG_PRINT(UARTprintf("Ca:");)
									for(i = 0; i < ISEs.Ca.size; i++)
									{DEBUG_PRINT(UARTprintf("\t%d", Ca_Cal_Check[i]);)}
							DEBUG_PRINT(UARTprintf("\n");)
						}

						if(Cal_failed == 0)
						{DEBUG_PRINT(UARTprintf("Calibration Passed!\n");)}
						else
						{DEBUG_PRINT(UARTprintf("Calibration Failed!\n");)}

#endif


						//
						// Check if we need to rerun a solution, only check if we haven't already repumped and don't have an abort command
						//
						if(solution == 3 && (RERUN_CALIBRATION) && (gui32Error & ABORT_ERRORS) == 0)
						{
							// Create variables to hold what solution needs to be repumped
							// If both need to be repumped run a whole calibration again
							// If only one needs to be repumped do that next before amp cleaning
							uint8_t Cal_1_repump = 0;
							uint8_t Cal_2_repump = 0;
							uint8_t Rinse_repump = 0;
							uint8_t Clean_repump = 0;

							if(Sols->pH_EEP_Cal_2 < 9)	// This is pH 9 Clean, Cal 5, and Cal 6 setup
							{
								uint8_t pH_H2_passed = 0;
								for(i = 0; i < ISEs.pH_H2.size; i++)
									pH_H2_passed += pH_H2_Cal_Check[i];

								if(pH_H2_passed == 0 && ISEs.pH_H2.size)
								{
									DEBUG_PRINT(UARTprintf("pH H2 failed calibration, don't have a third point to identify problem!\n");)
								}

								uint8_t pH_Cr_passed = 0;
								for(i = 0; i < ISEs.pH_Cr.size; i++)
									pH_Cr_passed += pH_Cr_Cal_Check[i];

								if(pH_Cr_passed == 0 && ISEs.pH_Cr.size > 0)
								{
									DEBUG_PRINT(UARTprintf("pH Cr failed calibration, looking to see if it was individual solution!\n");)
										//					float pH_TCor_Rinse = pH_EEP_Rinse + K_T_pH_Rinse * (T_Rinse - 25);

										//					uint8_t pH_Cal_Check_1R[3] = {0,0,0}, pH_Cal_Check_R2[3] = {0,0,0};
										uint8_t pH_Cr_Cal_Check_12 = 0;
									uint8_t pH_Cr_Cal_Check_C2 = 0;
									for(i = 0; i < ISEs.pH_Cr.size; i++)
									{
										float pH_Slope_12 = (ISE_mV_Cal_2[i + ISEs.pH_Cr.index] - pH_Cr_mV_Cal_1[i])/(pH_TCor_Cal_2 - pH_TCor_Cal_1);
										float pH_Slope_C2 = (pH_Cr_mV_Cal_2[i] - ISE_mV_Clean[i + ISEs.pH_Cr.index])/(pH_TCor_Cal_2 - pH_TCor_Clean);

										if(pH_Slope_12 <= PH_SLOPE_HIGH && pH_Slope_12 >= PH_SLOPE_LOW) // If slope is within range
											pH_Cr_Cal_Check_12++;
										if(pH_Slope_C2 <= PH_SLOPE_HIGH && pH_Slope_C2 >= PH_SLOPE_LOW) // If slope is within range
											pH_Cr_Cal_Check_C2++;
									}

									if(pH_Cr_Cal_Check_12 == 0)
									{
										DEBUG_PRINT(UARTprintf("pH sensors show Cal 1 needs to be repumped!\n");)
											Cal_1_repump = 1;
									}
									if(pH_Cr_Cal_Check_C2 == 0)
									{
										DEBUG_PRINT(UARTprintf("pH sensors show Clean needs to be repumped!\n");)
											Clean_repump = 1;
									}
								}

								uint8_t Ca_passed = 0;
								for(i = 0; i < ISEs.Ca.size; i++)
									Ca_passed += Ca_Cal_Check[i];

								if(Ca_passed == 0 && ISEs.Ca.size > 0)
								{
									DEBUG_PRINT(UARTprintf("Ca failed calibration, looking to see if it was individual solution!\n");)

										uint8_t Ca_Cal_Check_CR = 0, Ca_Cal_Check_R2 = 0, Ca_Cal_Check_C2;
									for(i = 0; i < ISEs.Ca.size; i++)
									{
										float Ca_Slope_CR = (Ca_mV_Rinse[i] - Ca_mV_Clean[i]) / (pCa_Rinse - pCa_Clean);
										float Ca_Slope_R2 = (Ca_mV_Cal_2[i] - Ca_mV_Rinse[i]) / (pCa_Cal_2 - pCa_Rinse);
										float Ca_Slope_C2 = (Ca_mV_Cal_2[i] - Ca_mV_Clean[i]) / (pCa_Cal_2 - pCa_Clean);

										if(Ca_Slope_CR <= CA_SLOPE_HIGH && Ca_Slope_CR >= CA_SLOPE_LOW) // If slope is within range
											Ca_Cal_Check_CR++;
										if(Ca_Slope_R2 <= CA_SLOPE_HIGH && Ca_Slope_R2 >= CA_SLOPE_LOW) // If slope is within range
											Ca_Cal_Check_R2++;
										if(Ca_Slope_C2 <= CA_SLOPE_HIGH && Ca_Slope_C2 >= CA_SLOPE_LOW) // If slope is within range
											Ca_Cal_Check_C2++;
									}


									if(Ca_Cal_Check_CR == 0 && Ca_Cal_Check_R2 == 0)
									{
										DEBUG_PRINT(UARTprintf("Ca sensors shows Rinse needs to be repumped!\n");)
											Rinse_repump = 1;
									}
									if(Ca_Cal_Check_CR == 0 && Ca_Cal_Check_C2 == 0)
									{
										DEBUG_PRINT(UARTprintf("Ca sensors shows Clean needs to be repumped!\n");)
											Clean_repump = 1;
									}
									if(Ca_Cal_Check_R2 == 0 && Ca_Cal_Check_C2 == 0)
									{
										DEBUG_PRINT(UARTprintf("Ca sensors shows Cal 2 needs to be repumped!\n");)
											Cal_2_repump = 1;
									}
								}


								uint8_t TH_passed = 0;
								for(i = 0; i < ISEs.TH.size; i++)
									TH_passed += TH_Cal_Check[i];

								if(TH_passed == 0 && ISEs.TH.size > 0)
								{
									DEBUG_PRINT(UARTprintf("TH failed calibration, looking to see if it was individual solution!\n");)

										uint8_t TH_Cal_Check_CR = 0, TH_Cal_Check_R2 = 0, TH_Cal_Check_C2;
									for(i = 0; i < ISEs.TH.size; i++)
									{
										float TH_Slope_CR = (TH_mV_Rinse[i] - TH_mV_Clean[i]) / (pTH_Rinse - pTH_Clean);
										float TH_Slope_R2 = (TH_mV_Cal_2[i] - TH_mV_Rinse[i]) / (pTH_Cal_2 - pTH_Rinse);
										float TH_Slope_C2 = (TH_mV_Cal_2[i] - TH_mV_Clean[i]) / (pTH_Cal_2 - pTH_Clean);

										if(TH_Slope_CR <= TH_SLOPE_HIGH && TH_Slope_CR >= TH_SLOPE_LOW) // If slope is within range
											TH_Cal_Check_CR++;
										if(TH_Slope_R2 <= TH_SLOPE_HIGH && TH_Slope_R2 >= TH_SLOPE_LOW) // If slope is within range
											TH_Cal_Check_R2++;
										if(TH_Slope_C2 <= TH_SLOPE_HIGH && TH_Slope_C2 >= TH_SLOPE_LOW) // If slope is within range
											TH_Cal_Check_C2++;
									}


									if(TH_Cal_Check_CR == 0 && TH_Cal_Check_R2 == 0)
									{
										DEBUG_PRINT(UARTprintf("TH sensors shows Rinse needs to be repumped!\n");)
											Rinse_repump = 1;
									}
									if(TH_Cal_Check_CR == 0 && TH_Cal_Check_C2 == 0)
									{
										DEBUG_PRINT(UARTprintf("TH sensors shows Clean needs to be repumped!\n");)
											Clean_repump = 1;
									}
									if(TH_Cal_Check_R2 == 0 && TH_Cal_Check_C2 == 0)
									{
										DEBUG_PRINT(UARTprintf("TH sensors shows Cal 2 needs to be repumped!\n");)
											Cal_2_repump = 1;
									}
								}

								uint8_t NH4_passed = 0;
								for(i = 0; i < ISEs.NH4.size; i++)
									NH4_passed += NH4_Cal_Check[i];

								if(NH4_passed == 0 && ISEs.NH4.size > 0)
								{
									DEBUG_PRINT(UARTprintf("NH4 failed calibration, looking to see if it was individual solution!\n");)

										uint8_t NH4_Cal_Check_1R = 0, NH4_Cal_Check_R2 = 0, NH4_Cal_Check_12;
									for(i = 0; i < ISEs.NH4.size; i++)
									{
										float NH4_Slope_1R = (NH4_mV_Rinse[i] - NH4_mV_Cal_1[i]) / (pNH4_Rinse - pNH4_Cal_1);
										float NH4_Slope_R2 = (NH4_mV_Cal_2[i] - NH4_mV_Rinse[i]) / (pNH4_Cal_2 - pNH4_Rinse);
										float NH4_Slope_12 = (NH4_mV_Cal_2[i] - NH4_mV_Cal_1[i]) / (pNH4_Cal_2 - pNH4_Cal_1);

										if(NH4_Slope_1R <= NH4_SLOPE_HIGH && NH4_Slope_1R >= NH4_SLOPE_LOW) // If slope is within range
											NH4_Cal_Check_1R++;
										if(NH4_Slope_R2 <= NH4_SLOPE_HIGH && NH4_Slope_R2 >= NH4_SLOPE_LOW) // If slope is within range
											NH4_Cal_Check_R2++;
										if(NH4_Slope_12 <= NH4_SLOPE_HIGH && NH4_Slope_12 >= NH4_SLOPE_LOW) // If slope is within range
											NH4_Cal_Check_12++;
									}


									if(NH4_Cal_Check_1R == 0 && NH4_Cal_Check_R2 == 0)
									{
										DEBUG_PRINT(UARTprintf("NH4 sensors shows Rinse needs to be repumped!\n");)
											Rinse_repump = 1;
									}
									if(NH4_Cal_Check_1R == 0 && NH4_Cal_Check_12 == 0)
									{
										DEBUG_PRINT(UARTprintf("NH4 sensors shows Cal 1 needs to be repumped!\n");)
											Cal_1_repump = 1;
									}
									if(NH4_Cal_Check_R2 == 0 && NH4_Cal_Check_12 == 0)
									{
										DEBUG_PRINT(UARTprintf("NH4 sensors shows Cal 2 needs to be repumped!\n");)
											Cal_2_repump = 1;
									}
								}

								if(CalConductivitySlopeLow < COND_SLOPE_1_LOW || CalConductivitySlopeLow > COND_SLOPE_1_HIGH)	// If conductivity low slope failed calibration
								{
									// Conductivity low slope uses factory cal and Cal 1, assume factory cal worked so if low slope failed must be cal 1
									Cal_1_repump = 1;
									DEBUG_PRINT(UARTprintf("Conductivity low slope failed, showing Cal 1 needs to be repumped!\n");)
								}
								if(CalConductivitySlopeMid < COND_SLOPE_2_LOW || CalConductivitySlopeMid > COND_SLOPE_2_HIGH)	// If conductivity mid slope failed calibration
								{
									// Mid slope is determined from Cal 2 and Rinse, if low slope also failed probably a problem with Cal 2
									if(CalConductivitySlopeLow < COND_SLOPE_1_LOW || CalConductivitySlopeLow > COND_SLOPE_1_HIGH)	// If conductivity low slope failed calibration
									{
										Cal_1_repump = 1;
										DEBUG_PRINT(UARTprintf("Conducitivity low and mid slopes failed, showing Cal 1 needs to be repumped!\n");)
									}
									else	// If low slope passed but mid slope failed must be a problem with rinse
									{
//										if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean && ISEs.Config != PH_CL_CART)
//										{
//											Clean_repump = 1;
//											DEBUG_PRINT(UARTprintf("Conductivity mid slope failed, showing Clean needs to be repumped!\n");)
//										}
//										else
//										{
											Rinse_repump = 1;
											DEBUG_PRINT(UARTprintf("Conductivity mid slope failed, showing Rinse needs to be repumped!\n");)
//										}
									}
								}
								if(CalConductivitySlopeHigh < COND_SLOPE_3_LOW || CalConductivitySlopeHigh > COND_SLOPE_3_HIGH)	// If conductivity high slope failed calibration
								{
									if(CalConductivitySlopeMid < COND_SLOPE_2_LOW || CalConductivitySlopeMid > COND_SLOPE_2_HIGH)	// If conductivity mid slope failed calibration
									{
//										if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean && ISEs.Config != PH_CL_CART)
//										{
//											Clean_repump = 1;
//											DEBUG_PRINT(UARTprintf("Conductivity mid and high slope failed, showing Clean needs to be repumped!\n");)
//										}
//										else
//										{
											Rinse_repump = 1;
											DEBUG_PRINT(UARTprintf("Conductivity mid and high slopes failed, showing Rinse needs to be repumped!\n");)
//										}
									}
									else
									{
										Cal_2_repump = 1;
										DEBUG_PRINT(UARTprintf("Conductivity high slope failed, showing Cal 2 needs to be repumped!\n");)
									}
								}
							}
							else	// This is not Cal 3
							{
								uint8_t pH_H2_passed = 0;
								for(i = 0; i < ISEs.pH_H2.size; i++)
									pH_H2_passed += pH_H2_Cal_Check[i];

								if(pH_H2_passed == 0 && ISEs.pH_H2.size)
								{
									DEBUG_PRINT(UARTprintf("pH H2 failed calibration, since this sensor is not linear down to Cal 2 don't have a third point to tell which solution had problem!\n");)
								}

								uint8_t pH_Cr_passed = 0;
								for(i = 0; i < ISEs.pH_Cr.size; i++)
									pH_Cr_passed += pH_Cr_Cal_Check[i];

								if(pH_Cr_passed == 0 && ISEs.pH_Cr.size > 0)
								{
									DEBUG_PRINT(UARTprintf("pH Cr failed calibration, looking to see if it was individual solution!\n");)
										//					float pH_TCor_Rinse = pH_EEP_Rinse + K_T_pH_Rinse * (T_Rinse - 25);

										//					uint8_t pH_Cal_Check_1R[3] = {0,0,0}, pH_Cal_Check_R2[3] = {0,0,0};
										uint8_t pH_Cr_Cal_Check_1R = 0;
									uint8_t pH_Cr_Cal_Check_R2 = 0;
									for(i = 0; i < ISEs.pH_Cr.size; i++)
									{
										float pH_Slope_1R = (pH_Cr_mV_Rinse[i] - pH_Cr_mV_Cal_1[i])/(pH_TCor_Rinse - pH_TCor_Cal_1);
										float pH_Slope_R2 = (pH_Cr_mV_Cal_2[i] - pH_Cr_mV_Rinse[i])/(pH_TCor_Cal_2 - pH_TCor_Rinse);

										if(pH_Slope_1R <= PH_SLOPE_HIGH && pH_Slope_1R >= PH_SLOPE_LOW) // If slope is within range
											pH_Cr_Cal_Check_1R++;
										if(pH_Slope_R2 <= PH_SLOPE_HIGH && pH_Slope_R2 >= PH_SLOPE_LOW) // If slope is within range
											pH_Cr_Cal_Check_R2++;
									}

									if(pH_Cr_Cal_Check_1R == 0)
									{
										DEBUG_PRINT(UARTprintf("pH sensors show Cal 1 needs to be repumped!\n");)
											Cal_1_repump = 1;
									}
									if(pH_Cr_Cal_Check_R2 == 0)
									{
										DEBUG_PRINT(UARTprintf("pH sensors show Cal 2 needs to be repumped!\n");)
											Cal_2_repump = 1;
									}
								}

								uint8_t Ca_passed = 0;
								for(i = 0; i < ISEs.Ca.size; i++)
									Ca_passed += Ca_Cal_Check[i];

								if(Ca_passed == 0 && ISEs.Ca.size > 0)
								{
									DEBUG_PRINT(UARTprintf("Ca failed calibration, looking to see if it was individual solution!\n");)

										uint8_t Ca_Cal_Check_1R = 0, Ca_Cal_Check_R2 = 0;
									for(i = 0; i < ISEs.Ca.size; i++)
									{
										float Ca_Slope_1R = (Ca_mV_Rinse[i] - Ca_mV_Cal_1[i]) / (pCa_Rinse - pCa_Cal_1);
										float Ca_Slope_R2 = (Ca_mV_Cal_2[i] - Ca_mV_Rinse[i]) / (pCa_Cal_2 - pCa_Rinse);

										if(Ca_Slope_1R <= CA_SLOPE_HIGH && Ca_Slope_1R >= CA_SLOPE_LOW) // If slope is within range
											Ca_Cal_Check_1R++;
										if(Ca_Slope_R2 <= CA_SLOPE_HIGH && Ca_Slope_R2 >= CA_SLOPE_LOW) // If slope is within range
											Ca_Cal_Check_R2++;
									}

									if(Ca_Cal_Check_1R == 0)
									{
										DEBUG_PRINT(UARTprintf("Ca sensors show Cal 1 needs to be repumped!\n");)
											Cal_1_repump = 1;
									}
									if(Ca_Cal_Check_R2 == 0)
									{
										DEBUG_PRINT(UARTprintf("Ca sensors show Cal 2 needs to be repumped!\n");)
#ifndef CALIBRATE_CA_1_R
											Cal_2_repump = 1;
#else
										DEBUG_PRINT(UARTprintf("Calibrating between Cal 1 and Rinse, ignoring Cal 2 repump!\n");)
#endif
									}
								}

								uint8_t TH_passed = 0;
								for(i = 0; i < ISEs.TH.size; i++)
									TH_passed += TH_Cal_Check[i];

								if(TH_passed == 0 && ISEs.TH.size > 0)
								{
									DEBUG_PRINT(UARTprintf("TH failed calibration, looking to see if it was individual solution!\n");)

										uint8_t TH_Cal_Check_1R = 0, TH_Cal_Check_R2 = 0;
									for(i = 0; i < ISEs.TH.size; i++)
									{
										float TH_Slope_1R = (TH_mV_Rinse[i] - TH_mV_Cal_1[i]) / (pTH_Rinse - pTH_Cal_1);
										float TH_Slope_R2 = (TH_mV_Cal_2[i] - TH_mV_Rinse[i]) / (pTH_Cal_2 - pTH_Rinse);

										if(TH_Slope_1R <= TH_SLOPE_HIGH && TH_Slope_1R >= TH_SLOPE_LOW) // If slope is within range
											TH_Cal_Check_1R++;
										if(TH_Slope_R2 <= TH_SLOPE_HIGH && TH_Slope_R2 >= TH_SLOPE_LOW) // If slope is within range
											TH_Cal_Check_R2++;
									}

									if(TH_Cal_Check_1R == 0)
									{
										DEBUG_PRINT(UARTprintf("TH sensors show Cal 1 needs to be repumped!\n");)
#ifndef CALIBRATE_TH_R_2
											Cal_1_repump = 1;
#else
										DEBUG_PRINT(UARTprintf("Calibrating between Rinse and Cal 2, ignoring Cal 1 repump!\n");)
#endif
									}
									if(TH_Cal_Check_R2 == 0)
									{
										DEBUG_PRINT(UARTprintf("TH sensors show Cal 2 needs to be repumped!\n");)
											Cal_2_repump = 1;
									}
								}

								uint8_t NH4_passed = 0;
								for(i = 0; i < ISEs.NH4.size; i++)
									NH4_passed += NH4_Cal_Check[i];

								if(NH4_passed == 0 && ISEs.NH4.size > 0)
								{
									DEBUG_PRINT(UARTprintf("NH4 failed calibration, looking to see if it was individual solution!\n");)

										//					float NH4_Slope_1R[3], NH4_Slope_R2[3];
										uint8_t NH4_Cal_Check_1R = 0, NH4_Cal_Check_R2 = 0;
									for(i = 0; i < ISEs.NH4.size; i++)
									{
										float NH4_Slope_1R = (NH4_mV_Rinse[i] - NH4_mV_Cal_1[i]) / (pNH4_Rinse - pNH4_Cal_1);
										float NH4_Slope_R2 = (NH4_mV_Clean[i] - NH4_mV_Rinse[i]) / (pNH4_Clean - pNH4_Rinse);

										if(NH4_Slope_1R <= NH4_SLOPE_HIGH && NH4_Slope_1R >= NH4_SLOPE_LOW) // If slope is within range
											NH4_Cal_Check_1R++;
										if(NH4_Slope_R2 <= NH4_SLOPE_HIGH && NH4_Slope_R2 >= NH4_SLOPE_LOW) // If slope is within range
											NH4_Cal_Check_R2++;
									}

									if(Sols->pH_EEP_Clean < 8.5)
									{
										if(NH4_Cal_Check_1R == 0)
										{
											DEBUG_PRINT(UARTprintf("NH4 sensors show Cal 1 needs to be repumped!\n");)
												Cal_1_repump = 1;
										}
										if(NH4_Cal_Check_R2 == 0)
										{
											if(NH4InClean)
											{
												Clean_repump = 1;
												DEBUG_PRINT(UARTprintf("NH4 sensors show Clean needs to be repumped!\n");)
											}

											else
											{
												DEBUG_PRINT(UARTprintf("NH4 sensors show Cal 2 needs to be repumped!\n");)
													Cal_2_repump = 1;
											}
										}
									}
									else
									{DEBUG_PRINT(UARTprintf("Don't have 3 points for NH4 sensor!\n");)}

								}

								if(CalConductivitySlopeLow < COND_SLOPE_1_LOW || CalConductivitySlopeLow > COND_SLOPE_1_HIGH)	// If conductivity low slope failed calibration
								{
									// Conductivity low slope uses factory cal and Cal 2, assume factory cal worked so if low slope failed must be cal 2
									Cal_2_repump = 1;
									DEBUG_PRINT(UARTprintf("Conductivity low slope failed, showing Cal 2 needs to be repumped!\n");)
								}
								if(CalConductivitySlopeMid < COND_SLOPE_2_LOW || CalConductivitySlopeMid > COND_SLOPE_2_HIGH)	// If conductivity mid slope failed calibration
								{
									// Mid slope is determined from Cal 2 and Rinse, if low slope also failed probably a problem with Cal 2
									if(CalConductivitySlopeLow < COND_SLOPE_1_LOW || CalConductivitySlopeLow > COND_SLOPE_1_HIGH)	// If conductivity low slope failed calibration
									{
										Cal_2_repump = 1;
										DEBUG_PRINT(UARTprintf("Conducitivity low and mid slopes failed, showing Cal 2 needs to be repumped!\n");)
									}
									else	// If low slope passed but mid slope failed must be a problem with rinse
									{
//										if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean && ISEs.Config != PH_CL_CART)
//										{
//											Clean_repump = 1;
//											DEBUG_PRINT(UARTprintf("Conductivity mid slope failed, showing Clean needs to be repumped!\n");)
//										}
//										else
//										{
											Rinse_repump = 1;
											DEBUG_PRINT(UARTprintf("Conductivity mid slope failed, showing Rinse needs to be repumped!\n");)
//										}
									}
								}
								if(CalConductivitySlopeHigh < COND_SLOPE_3_LOW || CalConductivitySlopeHigh > COND_SLOPE_3_HIGH)	// If conductivity high slope failed calibration
								{
									if(CalConductivitySlopeMid < COND_SLOPE_2_LOW || CalConductivitySlopeMid > COND_SLOPE_2_HIGH)	// If conductivity mid slope failed calibration
									{
//										if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean && ISEs.Config != PH_CL_CART)
//										{
//											Clean_repump = 1;
//											DEBUG_PRINT(UARTprintf("Conductivity mid and high slope failed, showing Clean needs to be repumped!\n");)
//										}
//										else
//										{
											Rinse_repump = 1;
											DEBUG_PRINT(UARTprintf("Conductivity mid and high slopes failed, showing Rinse needs to be repumped!\n");)
//										}
									}
									else
									{
										Cal_1_repump = 1;
										DEBUG_PRINT(UARTprintf("Conductivity high slope failed, showing Cal 1 needs to be repumped!\n");)
									}
								}
							}

							// Decide whether to rerun whole calibration, repump a single calibrant, or if nothing needs to be done
							if((Rinse_repump + Cal_1_repump + Cal_2_repump + Clean_repump) > 1)
							{
								if(Cal_Rerun == 0)	// If we aren't already rerunning a calibration
								{
									uint16_t No_of_cals = *((uint16_t *) MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_COMPLETED_CALS, 2));
									if(No_of_cals == 0xFFFF)
										No_of_cals = 0;

									No_of_cals++;
									MemoryWrite(PAGE_CARTRIDGE_INFO, OFFSET_COMPLETED_CALS, 2, (uint8_t *) &No_of_cals);

									DEBUG_PRINT(UARTprintf("Multiple calibrants need to be repumped, restarting calibration cycle!\n");)
									Cal_Rerun = 1;	// Set that next cal is a rerun,

//									// Save this cal number if sensors passed calibration
//									for(i = 0; i < 10; i++)
//										if(ISE_Cal_Check[i])
//											MemoryWrite(Cal_page, OFFSET_PH_1_LAST_P_CAL + i, 1, (uint8_t *) &Cal_Number);
//
//									// Save cal data to cartridge memory
//									MemoryWrite(Cal_page, OFFSET_T_CAL, 4, (uint8_t *) &T_Cal);
//									for(i = 0; i < 10; i++)
//									{
//										MemoryWrite(Cal_page, OFFSET_ISE_1_SLOPE + (i * 4), 4, (uint8_t *) &ISE_Slope_CalT[i]);
//										//							MemoryWrite(Cal_page, OFFSET_ISE_1_INT + (i * 4), 4, (uint8_t *) &ISE_Int[i]);
//										//							MemoryWrite(Cal_page, OFFSET_CR_ISE_1_RINSE + (i * 4), 4, (uint8_t *) &ISE_mV_Rinse[i]);
//									}
//
//									MemoryWrite(Cal_page, OFFSET_COND_R1_SLOPE, 4, (uint8_t *) &CalConductivitySlopeLow);
//									MemoryWrite(Cal_page, OFFSET_COND_R1_INT, 4, (uint8_t *) &CalConductivityKLow);
//									MemoryWrite(Cal_page, OFFSET_COND_R2_SLOPE, 4, (uint8_t *) &CalConductivitySlopeMid);
//									MemoryWrite(Cal_page, OFFSET_COND_R2_INT, 4, (uint8_t *) &CalConductivityKMid);
//									MemoryWrite(Cal_page, OFFSET_COND_R3_SLOPE, 4, (uint8_t *) &CalConductivitySlopeHigh);
//									MemoryWrite(Cal_page, OFFSET_COND_R3_INT, 4, (uint8_t *) &CalConductivityKHigh);
//
//									Calibration_Status = 0;
//									MemoryWrite(Cal_page, OFFSET_CAL_STATUS, 4, (uint8_t *) &Calibration_Status);
//									MemoryWrite(Cal_page, OFFSET_CAL_DATA_ZERO, 1, &Zero);
//
//									// Parse together the calibrated byte for the app reporting
//									if((gui32Error & CL_CLEANING_OUT_OF_RANGE) == 0)
//										App_Status |= (1 << APP_CHLORINE);
//									MemoryWrite(Cal_page, OFFSET_CALIBRATED_STATUS, 1, &App_Status);
//
//									MemoryWrite(Cal_page, OFFSET_CAL_LOG_K, 4, (uint8_t *) &log_K_Ca_Mg);
//
//									MemoryWrite(Cal_page, OFFSET_CR_ZERO, 1, &Zero);

									// Before restarting the calibration pump post-rinse as a rinsing step before jumping straight back into the calibration
									update_Status(STATUS_CALIBRATION, OPERATION_CAL_POSTCHECK);

									uint8_t Storage_Port;
#ifdef STORE_HIGH_CONC_CAL
									DEBUG_PRINT(UARTprintf("Pumping Cal 5 as Postrinse\n");)
									Storage_Port = V_CAL_2;
#else
									/*if(ISEs.Config == PH_CL_CART && Sols->Cond_EEP_Cal_2 > 900)	// pH only cartridge with pH 9 clean in place of Cal 2
									{
										DEBUG_PRINT(UARTprintf("Pumping Cal 2\n");)
											Storage_Port = V_CAL_2;
									}
									else */if((STORE_IN_CLEAN == 0 || (ISEs.Config == PH_CL_CART && STORE_PH6_CLEAN == 0)) && Sols->pH_EEP_Clean < 8.5)
									{
										DEBUG_PRINT(UARTprintf("Pumping Postrinse\n");)
											Storage_Port = V_RINSE;
									}
									else
									{
										DEBUG_PRINT(UARTprintf("Pumping Clean as Postrinse\n");)
											Storage_Port = V_CLEAN;
									}
#endif


									RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
									FindPossitionZeroPump();
									for (i = 0; i < Number_of_bubbles_Postrinse; i++) // Loop over air/solution cycle 3 times for single solution
									{
										RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
										if(i == (Number_of_bubbles_Postrinse - 1))
											PumpVolume(FW, PumpVol_air_bubble + PumpVol_Large_air_bubble, Speed_ISE, 0);
										else
											PumpVolume(FW, PumpVol_air_bubble, Speed_ISE, 0);
										userDelay(valve_delay_after_air, 0);
										RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
										if(i == 0 && BUBBLES_IN_TUBE)
											PumpVolume(FW, PumpVol_tube_prime + PumpVol_Solution, Speed_ISE, 0);
										else
											PumpVolume(FW, PumpVol_Solution, Speed_ISE, 0);
										if(i != (Number_of_bubbles_Postrinse - 1))
											userDelay(valve_delay, 0);
									}

									if(STORE_HUMID == 1)
									{
										DEBUG_PRINT(UARTprintf("Storing with air over all sensors!\n");)
											RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
										PumpVolume(FW, PumpVol_Solution + PumpVol_Solution_plug, Speed_ISE, 0);
										userDelay(valve_delay_after_air, 0);
										RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
										PumpVolume(FW, PumpVol_Solution, Speed_ISE, 0);
									}
									else if(STORE_FRIT_DRY == 1)
									{
										DEBUG_PRINT(UARTprintf("Storing with air over reference!\n");)
											//					userDelay(valve_delay, 1);
											//					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
											//					PumpStepperRunStepSpeed_AbortReady(FW, 3000, Speed_ISE);
											//					userDelay(valve_delay, 1);
											RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
										PumpVolume(FW, 134.4, Speed_ISE, 0);
										userDelay(valve_delay, 0);
									}
//									else if(STORE_AMPS_DRY == 1)
//									{
//										//					DEBUG_PRINT(UARTprintf("Storing air bubble over amperometrics only!\n");)
//										//					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
//										//					PumpStepperRunStepSpeed_AbortReady(FW, 2000, Speed_ISE);
//										//					userDelay(valve_delay, 1);
//										//					RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);
//										//					PumpStepperRunStepSpeed_AbortReady(FW, 11360, Speed_ISE);
//										//					userDelay(valve_delay, 1);
//
//										DEBUG_PRINT(UARTprintf("Storing air bubble over amperometrics only!\n");)
//											RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
//										PumpVolume(FW, PumpVol_air_bubble, Speed_ISE, 0);
//										userDelay(valve_delay_after_air, 0);
//										RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
//										PumpVolume(FW, 117.6, Speed_ISE, 0);
//										userDelay(valve_delay, 0);
//
//										// Set RE and CE floating and close RE/CE loop
//										IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
//										IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
//
//										// 10.7 uApp R = 309k + 499k = 808k
//										IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
//										IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);
//
//										WaveGenSet(1);
//										userDelay(100, 0);
//										uint8_t signal_doubled = 0;
//										uint8_t checkstep = 0;
//
//										float CondReading = ConductivityMovingAvg(COND_FREQ);
//										float OldReading = CondReading;
//
//										DEBUG_PRINT(UARTprintf("Reading\t%d\n", (int) CondReading);)
//										while(signal_doubled == 0 && checkstep <= 60)
//										{
//											PumpVolume(FW, 2.75, Speed_ISE, 0);
//											checkstep++;
//
//											CondReading = ConductivityMovingAvg(COND_FREQ);
//											DEBUG_PRINT(UARTprintf("Reading\t%d\n", (int) CondReading);)
//
//											if(CondReading/OldReading > 2)
//											{
//												DEBUG_PRINT(UARTprintf("Conductivity signal doubled! Found storage location!\n");)
//													signal_doubled = 1;
//											}
//											OldReading = CondReading;
//
//
//										}
//										if(checkstep > 37)
//										{
//											DEBUG_PRINT(UARTprintf("Didn't find storage location!\n");)
//												PumpVolume(FW, 33.6, Speed_ISE, 0);
//										}
//
//										WaveGenSet(0);
//									}
									else if(MEASURE_POSTRINSE_CAL == 0)
									{
										DEBUG_PRINT(UARTprintf("Storing covering everyting!\n");)
											PumpVolume(FW, PumpVol_Solution_plug, Speed_ISE, 0);
										if(FLOOD_TO_STORE == 0)
										{
											userDelay(valve_delay, 0);
											RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
										}
										PumpVolume(FW, PumpVol_air_plug, Speed_ISE, 0);
									}

									SleepValve();

									// Push air back into rinse port before moving to next solution
									if(BUBBLES_IN_TUBE)
									{
										RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
										//									if((STORE_IN_CLEAN == 0 || (ISEs.Config == PH_CL_CART && STORE_PH6_CLEAN == 0)) && Sols->pH_EEP_Clean < 8.5)
										PumpVolume(FW, PumpVol_tube_bubble * 4, Speed_ISE, 0);
										//									else
										//										PumpVolume(FW, PumpVol_tube_bubble * 6, Speed_ISE, 0);
										userDelay(valve_delay_after_air, 0);
										RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
										PumpVolume(BW, PumpVol_tube_bubble + 13.77, Speed_Slow, 0);
										PumpVolume(FW, 13.77, Speed_Slow, 0);
										userDelay(valve_delay, 0);
										SleepValve();
									}

									break;	// Break for loop keeping me in solution case statement and restart calibration
								}
								else	// We are rerunning a calibration, reset flag after check
									Cal_Rerun = 0;
							}
							else if(Rinse_repump == 1)
							{
								uint8_t Repump_times = *(MemoryRead(PAGE_SOLUTIONS, OFFSET_REPUMP_RINSE, 1));
								if(Repump_times == 0xFF)
									Repump_times = 0;
								Repump_times++;
								MemoryWrite(PAGE_SOLUTIONS, OFFSET_REPUMP_RINSE, 1, &Repump_times);

								if(Repump_times <= 5)
								{
									DEBUG_PRINT(UARTprintf("Only Rinse needs to be repumped, have repumped %d time(s)!\n", Repump_times);)
										calibrants_to_pump++;
									Cal_Order[4] = V_RINSE;
								}
								else
								{
									DEBUG_PRINT(UARTprintf("Already repumped Rinse 5 times, needs to be repumped but skipping!\n");)
										gui32Error |= CAL_REPUMP_FAIL;
								}
							}
							else if(Cal_1_repump == 1)
							{
								uint8_t Repump_times = *(MemoryRead(PAGE_SOLUTIONS, OFFSET_REPUMP_CAL_1, 1));
								if(Repump_times == 0xFF)
									Repump_times = 0;
								Repump_times++;
								MemoryWrite(PAGE_SOLUTIONS, OFFSET_REPUMP_CAL_1, 1, &Repump_times);

								if(Repump_times <= 5)
								{
									DEBUG_PRINT(UARTprintf("Only Cal 1 needs to be repumped, have repumped %d time(s)!\n", Repump_times);)
										calibrants_to_pump++;
									Cal_Order[4] = V_CAL_1;
								}
								else
								{
									DEBUG_PRINT(UARTprintf("Already repumped Cal 1 5 times, needs to be repumped but skipping!\n");)
										gui32Error |= CAL_REPUMP_FAIL;
								}
							}
							else if(Cal_2_repump == 1)
							{
								uint8_t Repump_times = *(MemoryRead(PAGE_SOLUTIONS, OFFSET_REPUMP_CAL_2, 1));
								if(Repump_times == 0xFF)
									Repump_times = 0;
								Repump_times++;
								MemoryWrite(PAGE_SOLUTIONS, OFFSET_REPUMP_CAL_2, 1, &Repump_times);

								if(Repump_times <= 5)
								{
									DEBUG_PRINT(UARTprintf("Only Cal 2 needs to be repumped, have repumped %d time(s)!\n", Repump_times);)
										calibrants_to_pump++;
									Cal_Order[4] = V_CAL_2;
								}
								else
								{
									DEBUG_PRINT(UARTprintf("Already repumped Cal 2 5 times, need to be repumped but skipping!\n");)
										gui32Error |= CAL_REPUMP_FAIL;
								}
							}
							else if(Clean_repump == 1)
							{
								uint8_t Repump_times = *(MemoryRead(PAGE_SOLUTIONS, OFFSET_REPUMP_CLEAN, 1));
								if(Repump_times == 0xFF)
									Repump_times = 0;
								Repump_times++;
								MemoryWrite(PAGE_SOLUTIONS, OFFSET_REPUMP_CLEAN, 1, &Repump_times);

								if(Repump_times <= 5)
								{
									DEBUG_PRINT(UARTprintf("Only Clean needs to be repumped, have repumped %d time(s)!\n", Repump_times);)
										calibrants_to_pump++;
									Cal_Order[4] = V_CLEAN;
								}
								else
								{
									DEBUG_PRINT(UARTprintf("Already repumped Clean 5 times, need to be repumped but skipping!\n");)
										gui32Error |= CAL_REPUMP_FAIL;
								}
							}
							else
							{
								DEBUG_PRINT(UARTprintf("Nothing needs to be repumped, calibration must have passed!\n");)
							}
						}
					}
					else
					{DEBUG_PRINT(UARTprintf("Calibration Aborted!\n");)}

					// If the calibration needs to be reran the for loop will have already been broken out of
					// so turn off cal rerun here so we don't get stuck in an infinite loop
					Cal_Rerun = 0;
				}	// if last solution check repump
			}	// For solution

			if(Cal_Rerun == 1 && (gui32Error & ABORT_ERRORS) == 0)	// flag is set that cal needs to be re-ran
				break;	// Break out of calibration case

			//
			// Prime and reset buffers and titrants
			//
			uint8_t Storage_Port;
#ifdef STORE_HIGH_CONC_CAL
			DEBUG_PRINT(UARTprintf("Pumping Cal 5 as Postrinse\n");)
			Storage_Port = V_CAL_2;
#else
			/*if(ISEs.Config == PH_CL_CART && Sols->Cond_EEP_Cal_2 > 900)	// pH only cartridge with pH 9 clean in place of Cal 2
				Storage_Port = V_CAL_2;
			else */if((STORE_IN_CLEAN == 0 || (ISEs.Config == PH_CL_CART && STORE_PH6_CLEAN == 0)) && Sols->pH_EEP_Clean < 8.5)
				Storage_Port = V_RINSE;
			else
				Storage_Port = V_CLEAN;
#endif

			PrintTime();
			//#ifdef TESTING_MODE
			//			if((gui32Error & ABORT_ERRORS) == 0 && PRIME_BUFFERS_CAL && MEASURE_POSTRINSE_CAL == 0)
			//#else
			if((gui32Error & ABORT_ERRORS) == 0 && PRIME_BUFFERS_CAL && BUBBLES_IN_TUBE)
				//#endif
			{
				//				uint16_t Steps_clear = 45000;

				Speed_BufferPrime = 3000;


				uint8_t FirstPassedCal = Calibration_Status & 1;	// Flag to check if this is the first passed calibration, if it is increase the size of the buffer prime
				if(FirstPassedCal != 0)
				{
					for(i = 1; i < Cal_Number; i++)
					{
						if(*MemoryRead(Find_Cal_page(i), OFFSET_CAL_STATUS, 4) & 1)
						{
							DEBUG_PRINT(UARTprintf("Found a passed calibration\n");)
							FirstPassedCal = 0;	// Set flag to 0
							break;	// Break out of for loop
						}
					}
				}

				if(PRIME_POUCH_TUBES && (FirstPassedCal == 1 || PrimePouchTubes == 1))
				{
					DEBUG_PRINT(UARTprintf("Increase size of buffer prime\n");)
					PumpVol_tube_prime_buffers = 240;
				}

				// Prime all the solutions that have air bubbles in their tubes, solution moves back in tube over time, trying to fix this
				// 8/3/2021 want to prime and reset them all seperately to try to prevent contamination spreading through the valve while they're all there
				DEBUG_PRINT(UARTprintf("Resetting B1...\n");)
				RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);
				PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_BufferPrime, 1);
				userDelay(valve_delay, 1);
				RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
				PumpVolume(FW, PumpVol_air_bubble * 1.5, Speed_BufferPrime, 1);
				userDelay(valve_delay_after_air, 1);
				RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to air
				PumpVolume(BW, PumpVol_tube_bubble + 13.77, Speed_ISE, 1);
				PumpVolume(FW, 13.77, Speed_ISE, 1);
				userDelay(valve_delay, 1);

				if(ISEs.Config != PH_CL_CART)
				{
					DEBUG_PRINT(UARTprintf("Resetting T1...\n");)
										RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);
					PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_BufferPrime, 1);
					userDelay(valve_delay, 1);

					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
					PumpVolume(FW, PumpVol_air_bubble * 1.5, Speed_BufferPrime, 1);
					userDelay(valve_delay_after_air, 1);

					RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to air
					PumpVolume(BW, PumpVol_tube_bubble + 13.77, Speed_ISE, 1);
					PumpVolume(FW, 13.77, Speed_ISE, 1);
					userDelay(valve_delay, 1);
				}

#ifndef H2_CALIBRATE_CAL6_B2_MIX
				DEBUG_PRINT(UARTprintf("Resetting B2...\n");)
				RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);
				PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_BufferPrime, 1);
				userDelay(valve_delay, 1);
				RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
				PumpVolume(FW, PumpVol_air_bubble * 1.5, Speed_BufferPrime, 1);
				userDelay(valve_delay_after_air, 1);
				RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);		// Move valve to air
				PumpVolume(BW, PumpVol_tube_bubble + 13.77, Speed_ISE, 1);
				PumpVolume(FW, 13.77, Speed_ISE, 1);
				userDelay(valve_delay, 1);
#endif

				DEBUG_PRINT(UARTprintf("Resetting C2...\n");)
				RunValveToPossition_Bidirectional_AbortReady(V_C2, VALVE_STEPS_PER_POSITION);
				PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_BufferPrime, 1);
				userDelay(valve_delay, 1);
				RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
				PumpVolume(FW, PumpVol_air_bubble * 1.5, Speed_BufferPrime, 1);
				userDelay(valve_delay_after_air, 1);
				RunValveToPossition_Bidirectional_AbortReady(V_C2, VALVE_STEPS_PER_POSITION);		// Move valve to air
				PumpVolume(BW, PumpVol_tube_bubble + 13.77, Speed_ISE, 1);
				PumpVolume(FW, 13.77, Speed_ISE, 1);
				userDelay(valve_delay, 1);

				//#ifdef BLEACH_POUCH
				//					// Pump in clean over amperometrics for cleaning
				//					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
				//					FindPossitionZeroPump();
				//					userDelay(valve_delay_after_air, 1);
				//					DEBUG_PRINT(UARTprintf("Pumping bleach!\n");)
				//					RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);
				//
				//					if(BUBBLES_IN_TUBE)
				//						PumpVolume(FW, PumpVol_tube_bubble, Speed_Slow, 1);
				//					PumpVolume(FW, PumpVol_air_bubble * 2, Speed_Slow, 1);
				//					userDelay(valve_delay, 1);
				//					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
				//					PumpVolume(FW, PumpVol_air_bubble + PumpVol_tube_bubble, Speed_Slow, 1);
				//					userDelay(valve_delay_after_air, 1);
				//					RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);
				//					PumpVolume(BW, PumpVol_tube_bubble, Speed_Slow, 1);
				//
				//					SleepValve();
				//#endif

				// Push a plug of rinse after buffers to help dilute anything left behind, Simon had his waste port in the valve get blocked (we think it was salt but didn't see) which caused
				// the pump tube to fall off and dump water inside the Roam
				RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
				PumpVolume(FW, PumpVol_Solution, Speed_ISE, 1);	// Slowed down this pumping so Bleach
				userDelay(valve_delay, 1);

				RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
				//				PumpVolume(FW, 756, Speed_ISE, 1);
				PumpVolume(FW, 200, Speed_Slow, 1);
				PumpVolume(FW, 556, Speed_ISE, 1);
				userDelay(valve_delay_after_air, 1);
			}

////#ifdef TESTING_MODE
////			if((gui32Error & ABORT_ERRORS) == 0 && PRIME_BUFFERS_CAL && MEASURE_POSTRINSE_CAL == 0)
////#else
//			if((gui32Error & ABORT_ERRORS) == 0 && PRIME_BUFFERS_CAL && BUBBLES_IN_TUBE)
////#endif
//			{
////				uint16_t Steps_clear = 45000;
//
//				if(PRIME_POUCH_TUBES && (Cal_Number == 1 || PrimePouchTubes == 1))
//				{
//					PumpVol_tube_prime_buffers = 240;
//				}
//
//				// Prime all the solutions that have air bubbles in their tubes, solution moves back in tube over time, trying to fix this
//				// 8/3/2021 want to prime and reset them all seperately to try to prevent contamination spreading through the valve while they're all there
//				if(ISEs.Config != PH_CL_CART)
//				{
//					DEBUG_PRINT(UARTprintf("Resetting T1...\n");)
//					RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);
//					PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_BufferPrime, 1);
//					userDelay(valve_delay, 1);
//
//					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
//					PumpVolume(FW, PumpVol_air_bubble * 1.5, Speed_BufferPrime, 1);
//					userDelay(valve_delay_after_air, 1);
//
//					RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to air
//					PumpVolume(BW, PumpVol_tube_bubble + 13.77, Speed_Slow, 1);
//					PumpVolume(FW, 13.77, Speed_Slow, 1);
//					userDelay(valve_delay, 1);
//				}
//
//				DEBUG_PRINT(UARTprintf("Resetting B1...\n");)
//				RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);
//				PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_BufferPrime, 1);
//				userDelay(valve_delay, 1);
//				RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
//				PumpVolume(FW, PumpVol_air_bubble * 1.5, Speed_BufferPrime, 1);
//				userDelay(valve_delay_after_air, 1);
//				RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to air
//				PumpVolume(BW, PumpVol_tube_bubble + 13.77, Speed_Slow, 1);
//				PumpVolume(FW, 13.77, Speed_Slow, 1);
//				userDelay(valve_delay, 1);
//
//				DEBUG_PRINT(UARTprintf("Resetting B2...\n");)
//				RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);
//				PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_BufferPrime, 1);
//				userDelay(valve_delay, 1);
//				RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
//				PumpVolume(FW, PumpVol_air_bubble * 1.5, Speed_BufferPrime, 1);
//				userDelay(valve_delay_after_air, 1);
//				RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);		// Move valve to air
//				PumpVolume(BW, PumpVol_tube_bubble + 13.77, Speed_Slow, 1);
//				PumpVolume(FW, 13.77, Speed_Slow, 1);
//				userDelay(valve_delay, 1);
//
//				DEBUG_PRINT(UARTprintf("Resetting C2...\n");)
//				RunValveToPossition_Bidirectional_AbortReady(V_C2, VALVE_STEPS_PER_POSITION);
//				PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_BufferPrime, 1);
//				userDelay(valve_delay, 1);
//				RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
//				PumpVolume(FW, PumpVol_air_bubble * 1.5, Speed_BufferPrime, 1);
//				userDelay(valve_delay_after_air, 1);
//				RunValveToPossition_Bidirectional_AbortReady(V_C2, VALVE_STEPS_PER_POSITION);		// Move valve to air
//				PumpVolume(BW, PumpVol_tube_bubble + 13.77, Speed_Slow, 1);
//				PumpVolume(FW, 13.77, Speed_Slow, 1);
//				userDelay(valve_delay, 1);
//
////#ifdef BLEACH_POUCH
////					// Pump in clean over amperometrics for cleaning
////					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
////					FindPossitionZeroPump();
////					userDelay(valve_delay_after_air, 1);
////					DEBUG_PRINT(UARTprintf("Pumping bleach!\n");)
////					RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);
////
////					if(BUBBLES_IN_TUBE)
////						PumpVolume(FW, PumpVol_tube_bubble, Speed_Slow, 1);
////					PumpVolume(FW, PumpVol_air_bubble * 2, Speed_Slow, 1);
////					userDelay(valve_delay, 1);
////					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
////					PumpVolume(FW, PumpVol_air_bubble + PumpVol_tube_bubble, Speed_Slow, 1);
////					userDelay(valve_delay_after_air, 1);
////					RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);
////					PumpVolume(BW, PumpVol_tube_bubble, Speed_Slow, 1);
////
////					SleepValve();
////#endif
//
//				// Push a plug of rinse after buffers to help dilute anything left behind, Simon had his waste port in the valve get blocked (we think it was salt but didn't see) which caused
//				// the pump tube to fall off and dump water inside the Roam
//				RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
//				PumpVolume(FW, PumpVol_Solution, Speed_Slow, 1);	// Slowed down this pumping so Bleach
//				userDelay(valve_delay_after_air, 1);
//
//				RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
////				PumpVolume(FW, 756, Speed_ISE, 1);
//				PumpVolume(FW, 200, Speed_Slow, 1);
//				PumpVolume(FW, 556, Speed_ISE, 1);
//				userDelay(valve_delay_after_air, 1);
//			}

			PrimePouchTubes = 0;

			//
			// Flow chart teal section, ISE post check, PostRinse
			//
			//			float pH_mV_PostRinse[3] = {0,0,0}, Ca_mV_PostRinse[2] = {0,0}, TH_mV_PostRinse[2] = {0,0};
			//			float NH4_mV_PostRinse[3] = {0,0,0};
			float ISE_mV_PostRinse[10] = {0,0,0,0,0,0,0,0,0,0};
//#ifdef PRINT_UART
//			float *pH_H2_mV_PostRinse = &ISE_mV_PostRinse[ISEs.pH_H2.index];
//			float *pH_Cr_mV_PostRinse = &ISE_mV_PostRinse[ISEs.pH_Cr.index];
//			float *TH_mV_PostRinse = &ISE_mV_PostRinse[ISEs.TH.index];
//			float *NH4_mV_PostRinse = &ISE_mV_PostRinse[ISEs.NH4.index];
//			float *Ca_mV_PostRinse = &ISE_mV_PostRinse[ISEs.Ca.index];
//#endif
			float T_PostRinse = 25;

			PrintTime();
//			if((gui32Error & ABORT_ERRORS) == 0)
			{
				update_Cal(Cal_Number);
				update_Status(STATUS_CALIBRATION, OPERATION_CAL_POSTCHECK);

#ifdef PRINT_UART
				if(Storage_Port == V_CAL_2)	// pH only cartridge with pH 9 clean in place of Cal 2
					DEBUG_PRINT(UARTprintf("Storing from Cal 2 port\n");)
				else if(Storage_Port == V_RINSE)
					DEBUG_PRINT(UARTprintf("Pumping Postrinse... \n");)
				else
					DEBUG_PRINT(UARTprintf("Storing in Clean... \n");)
#endif

				RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
				FindPossitionZeroPump();
				for (i = 0; i < Number_of_bubbles_Postrinse; i++) // Loop over air/solution cycle 3 times for single solution
				{
					RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					if(i == (Number_of_bubbles_Postrinse - 1))
						PumpVolume(FW, PumpVol_air_bubble + PumpVol_Large_air_bubble, Speed_ISE, 0);
					else
						PumpVolume(FW, PumpVol_air_bubble, Speed_ISE, 0);
					userDelay(valve_delay_after_air, 0);
					RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
					if(i == 0 && BUBBLES_IN_TUBE)
						PumpVolume(FW, PumpVol_tube_prime + PumpVol_Solution, Speed_ISE, 0);
					else
						PumpVolume(FW, PumpVol_Solution, Speed_ISE, 0);
					if(i != (Number_of_bubbles_Postrinse - 1))
						userDelay(valve_delay, 0);
				}

				if(MEASURE_POSTRINSE_CAL)
				{
					PumpVolume(FW, PumpVol_Solution_plug, Speed_ISE, 0);
					if(STORE_HUMID || STORE_FRIT_DRY /*|| STORE_AMPS_DRY*/)
					{
						userDelay(valve_delay, 0);
						RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					}
					else if(FLOOD_TO_STORE == 0)
					{
						userDelay(valve_delay, 0);
						RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					}
					PumpVolume(FW, PumpVol_air_plug, Speed_ISE, 0);

					CollectISEmV(ISE_mV_PostRinse, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);

					T_PostRinse = MeasureTemperature(1);

					// Set RE and CE floating and close RE/CE loop
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);

					MemoryWrite(Cal_page, OFFSET_CR_ISE_1_POST, 40, (uint8_t *) ISE_mV_PostRinse);
					MemoryWrite(Cal_page, OFFSET_CR_T_POSTRINSE, 4, (uint8_t *) &T_PostRinse);

					if(STORE_HUMID || STORE_FRIT_DRY /*|| STORE_AMPS_DRY*/)
					{
						RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
						PumpVolume(FW, PumpVol_Solution, Speed_ISE, 0);
						userDelay(valve_delay, 0);
					}
				}

				if(STORE_HUMID == 1)
				{
					DEBUG_PRINT(UARTprintf("Storing with air over all sensors!\n");)
					RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					PumpVolume(FW, PumpVol_Solution + PumpVol_Solution_plug, Speed_ISE, 0);
					userDelay(valve_delay_after_air, 0);
					RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
					PumpVolume(FW, PumpVol_Solution, Speed_ISE, 0);
				}
				else if(STORE_FRIT_DRY == 1)
				{
					DEBUG_PRINT(UARTprintf("Storing with air over reference!\n");)
//					userDelay(valve_delay, 1);
//					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
//					PumpStepperRunStepSpeed_AbortReady(FW, 3000, Speed_ISE);
//					userDelay(valve_delay, 1);
					RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
					PumpVolume(FW, 134.4, Speed_ISE, 0);
					userDelay(valve_delay, 0);
				}
//				else if(STORE_AMPS_DRY == 1)
//				{
////					DEBUG_PRINT(UARTprintf("Storing air bubble over amperometrics only!\n");)
////					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
////					PumpStepperRunStepSpeed_AbortReady(FW, 2000, Speed_ISE);
////					userDelay(valve_delay, 1);
////					RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);
////					PumpStepperRunStepSpeed_AbortReady(FW, 11360, Speed_ISE);
////					userDelay(valve_delay, 1);
//
//					DEBUG_PRINT(UARTprintf("Storing air bubble over amperometrics only!\n");)
//					RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
//					PumpVolume(FW, PumpVol_air_bubble, Speed_ISE, 0);
//					userDelay(valve_delay_after_air, 0);
//					RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
//					PumpVolume(FW, 117.6, Speed_ISE, 0);
//					userDelay(valve_delay, 0);
//
//					// Set RE and CE floating and close RE/CE loop
//					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
//					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
//
//					// 10.7 uApp R = 309k + 499k = 808k
//					IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
//					IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);
//
//					WaveGenSet(1);
//					userDelay(100, 0);
//					uint8_t signal_doubled = 0;
//					uint8_t checkstep = 0;
//
//					float CondReading = ConductivityMovingAvg(COND_FREQ);
//					float OldReading = CondReading;
//
//					DEBUG_PRINT(UARTprintf("Reading\t%d\n", (int) CondReading);)
//					while(signal_doubled == 0 && checkstep <= 60)
//					{
//						PumpVolume(FW, 2.75, Speed_ISE, 0);
//						checkstep++;
//
//						CondReading = ConductivityMovingAvg(COND_FREQ);
//						DEBUG_PRINT(UARTprintf("Reading\t%d\n", (int) CondReading);)
//
//						if(CondReading/OldReading > 2)
//						{
//							DEBUG_PRINT(UARTprintf("Conductivity signal doubled! Found storage location!\n");)
//							signal_doubled = 1;
//						}
//						OldReading = CondReading;
//
//
//					}
//					if(checkstep > 37)
//					{
//						DEBUG_PRINT(UARTprintf("Didn't find storage location!\n");)
//						PumpVolume(FW, 33.6, Speed_ISE, 0);
//					}
//
//					WaveGenSet(0);
//				}
				else if(MEASURE_POSTRINSE_CAL == 0)
				{
					DEBUG_PRINT(UARTprintf("Storing covering everyting!\n");)
					PumpVolume(FW, PumpVol_Solution_plug, Speed_ISE, 0);
					if(FLOOD_TO_STORE == 0)
					{
						userDelay(valve_delay, 0);
						RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					}
					PumpVolume(FW, PumpVol_air_plug, Speed_ISE, 0);
				}

				SleepValve();
			}

			// Push air back into rinse port before moving to next solution
			if(BUBBLES_IN_TUBE)
			{
				RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
//				if((STORE_IN_CLEAN == 0 || (ISEs.Config == PH_CL_CART && STORE_PH6_CLEAN == 0)) && Sols->pH_EEP_Clean < 8.5)
					PumpVolume(FW, PumpVol_tube_bubble * 4, Speed_ISE, 0);
//				else
//					PumpVolume(FW, PumpVol_tube_bubble * 6, Speed_ISE, 0);
				userDelay(valve_delay_after_air, 0);
				RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
				PumpVolume(BW, PumpVol_tube_bubble + 13.77, Speed_Slow, 0);
				PumpVolume(FW, 13.77, Speed_Slow, 0);
				userDelay(valve_delay, 0);
				SleepValve();
			}

//			Calibration_Status = 0;
//			uint8_t App_Status = 0;
//			if((gui32Error & ABORT_ERRORS) == 0)
//			{
//				float ISE_Slope_CalT[10] = {0,0,0,0,0,0,0,0,0,0};
//				float *pH_H2_Slope_CalT = &ISE_Slope_CalT[ISEs.pH_H2.index];
//				float *pH_Cr_Slope_CalT = &ISE_Slope_CalT[ISEs.pH_Cr.index];
//				float *TH_Slope_CalT = &ISE_Slope_CalT[ISEs.TH.index];
//				float *NH4_Slope_CalT = &ISE_Slope_CalT[ISEs.NH4.index];
//				float *Ca_Slope_CalT = &ISE_Slope_CalT[ISEs.Ca.index];
//
////				float ISE_Int[10] = {0,0,0,0,0,0,0,0,0,0};
////				float *pH_H2_Int = &ISE_Int[ISEs.pH_H2.index];
////				float *pH_Cr_Int = &ISE_Int[ISEs.pH_Cr.index];
////				float *TH_Int = &ISE_Int[ISEs.TH.index];
////				float *NH4_Int = &ISE_Int[ISEs.NH4.index];
////				float *Ca_Int = &ISE_Int[ISEs.Ca.index];
//
//				uint8_t ISE_Cal_Check[10] = {0,0,0,0,0,0,0,0,0,0};
//				uint8_t *pH_H2_Cal_Check = &ISE_Cal_Check[ISEs.pH_H2.index];
//				uint8_t *pH_Cr_Cal_Check = &ISE_Cal_Check[ISEs.pH_Cr.index];
//				uint8_t *TH_Cal_Check = &ISE_Cal_Check[ISEs.TH.index];
//				uint8_t *NH4_Cal_Check = &ISE_Cal_Check[ISEs.NH4.index];
//				uint8_t *Ca_Cal_Check = &ISE_Cal_Check[ISEs.Ca.index];
//
//				// Define here so I can set to a sensor that passed calibration, doesn't matter which one just has to be one that passed, will redecide which one is best later based on linearity
//				uint8_t L_pH_H2_Chosen = 0;
//				uint8_t L_pH_Cr_Chosen = 0;
//				uint8_t L_Ca_Chosen = 0;
//				uint8_t L_TH_Chosen = 0;
//				uint8_t L_NH4_Chosen = 0;
//
//				float T_Cal = (T_Cal_1 + T_Cal_2 + T_Rinse) / 3;
//
//				//
//				// pH H2 Calibration
//				//
//				float pH_TCor_Rinse = Calc_pH_TCor(Sols->pH_EEP_Rinse, T_Cal, 25, 0, Sols->K_T_pH_Rinse);
//				float pH_TCor_Cal_1 = Calc_pH_TCor(Sols->pH_EEP_Cal_1, T_Cal, 25, 0, Sols->K_T_pH_Cal_1);
//				float pH_TCor_Cal_2 = Calc_pH_TCor(Sols->pH_EEP_Cal_2, T_Cal, 25, 0, Sols->K_T_pH_Cal_2);
//				float pH_TCor_Clean = Calc_pH_TCor(Sols->pH_EEP_Clean, T_Cal, 25, Sols->K_T_pH_Clean_Sq, Sols->K_T_pH_Clean_Ln);
////				float pH_TCor_Clean = Sols->pH_EEP_Clean + (Sols->K_T_pH_Clean_Sq * (pow(T_Cal, 2) - pow(25, 2)) + Sols->K_T_pH_Clean_Ln * (T_Cal - 25));	// Temperature corrected pH for Rinse
//
//#ifdef LINEAR_PH_CORR
//						float log_K_Ca_Mg = -5;
//#else
//						float log_K_Ca_Mg = LOG_K_CA_MG;
//#endif
//
//				// Values are concentrations, calculate rinse p-values
//				float pCa_Rinse, pTH_Rinse, pNH4_Rinse;
//				pCa_Rinse = Calc_pCa(Sols->Ca_EEP_Rinse, T_Cal, Sols->IS_RINSE);
//				pTH_Rinse = Calc_pTH(Sols->Ca_EEP_Rinse, Sols->TH_EEP_Rinse, log_K_Ca_Mg, T_Cal, Sols->IS_RINSE);
//				pNH4_Rinse = Calc_pNH4(Sols->NH4_EEP_Rinse, pH_TCor_Rinse, 0, T_Cal, Sols->IS_RINSE);
//
//				if(ISEs.Config == PH_CL_CART)
//					pH_TCor_Rinse = pH_TCor_Clean;
//
//				for(i = 0; i < ISEs.pH_H2.size; i++)
//				{
//#ifdef CALIBRATE_H2_IN_CLEAN
//					if(Sols->pH_EEP_Clean > 8.5 && Sols->pH_EEP_Cal_2 > 9)	// This is pH 9 Clean
//						pH_H2_Slope_CalT[i] = (pH_H2_mV_Rinse[i] - pH_H2_mV_Cal_1[i])/(pH_TCor_Rinse - pH_TCor_Cal_1);
//					else if(Sols->pH_EEP_Clean > 8.5 && Sols->pH_EEP_Cal_2 < 7)	// This is pH 9 Clean and Cal 5
//						pH_H2_Slope_CalT[i] = (pH_H2_mV_Cal_2[i] - pH_H2_mV_Cal_1[i])/(pH_TCor_Cal_2 - pH_TCor_Cal_1);
//					else
//						pH_H2_Slope_CalT[i] = (pH_H2_mV_Clean[i] - pH_H2_mV_Cal_1[i])/(pH_TCor_Clean - pH_TCor_Cal_1);
//#else
//					pH_H2_Slope_CalT[i] = (pH_H2_mV_Rinse[i] - pH_H2_mV_Cal_1[i])/(pH_TCor_Rinse - pH_TCor_Cal_1);
//#endif
//					float pH_H2_Int = pH_H2_mV_Cal_1[i] - (pH_H2_Slope_CalT[i] * pH_TCor_Cal_1);
//					MemoryWrite(Cal_page, OFFSET_ISE_1_INT + ((i + ISEs.pH_H2.index) * 4), 4, (uint8_t *) &pH_H2_Int);
//					if(pH_H2_Slope_CalT[i] <= PH_SLOPE_HIGH && pH_H2_Slope_CalT[i] >= PH_SLOPE_LOW) // If slope is within range
//					{
//						pH_H2_Cal_Check[i] = 1;
//						L_pH_H2_Chosen = i;
//						App_Status |= (1 << APP_ALKALINITY);
//					}
//				}
//
//				//
//				// pH Cr Calibration
//				//
//				for(i = 0; i < ISEs.pH_Cr.size; i++)
//				{
//					float pH_Cr_Int;
//					if(Sols->pH_EEP_Clean > 8.5 && Sols->pH_EEP_Cal_2 < 7)	// This is pH 9 Clean and Cal 5 setup
//					{
//						pH_Cr_Slope_CalT[i] = (pH_Cr_mV_Clean[i] - pH_Cr_mV_Cal_1[i])/(pH_TCor_Clean - pH_TCor_Cal_1);
//	//					pH_Cr_Int[i] = pH_Cr_mV_Cal_1[i] - (pH_Cr_Slope_CalT[i] * pH_TCor_Cal_1);
//						pH_Cr_Int = pH_Cr_mV_Cal_1[i] - (pH_Cr_Slope_CalT[i] * pH_TCor_Cal_1);
//					}
//					else
//					{
//						pH_Cr_Slope_CalT[i] = (pH_Cr_mV_Cal_2[i] - pH_Cr_mV_Cal_1[i])/(pH_TCor_Cal_2 - pH_TCor_Cal_1);
//	//					pH_Cr_Int[i] = pH_Cr_mV_Cal_1[i] - (pH_Cr_Slope_CalT[i] * pH_TCor_Cal_1);
//						pH_Cr_Int = pH_Cr_mV_Cal_1[i] - (pH_Cr_Slope_CalT[i] * pH_TCor_Cal_1);
//					}
//
//					MemoryWrite(Cal_page, OFFSET_ISE_1_INT + ((i + ISEs.pH_Cr.index) * 4), 4, (uint8_t *) &pH_Cr_Int);
//					if(pH_Cr_Slope_CalT[i] <= PH_SLOPE_HIGH && pH_Cr_Slope_CalT[i] >= PH_SLOPE_LOW) // If slope is within range
//					{
//						pH_Cr_Cal_Check[i] = 1;
//						L_pH_Cr_Chosen = i;
//						App_Status |= (1 << APP_PH);
//					}
//				}
//
//				//
//				// Ca Calibration
//				//
//
//				// Values are concentration, calculate p-values
//				float pCa_Cal_1, pCa_Cal_2, pCa_Clean;
//				pCa_Cal_1 = Calc_pCa(Sols->Ca_EEP_Cal_1, T_Cal, Sols->IS_CAL_1);
//				pCa_Cal_2 = Calc_pCa(Sols->Ca_EEP_Cal_2, T_Cal, Sols->IS_CAL_2);
//				pCa_Clean = Calc_pCa(Sols->Ca_EEP_Clean, T_Cal, Sols->IS_CLEAN);
//
//				for(i = 0; i < ISEs.Ca.size; i++)
//				{
//#ifdef LINEAR_PH_CORR
//					float Ca_mpH, Ca_Int;
//					if(Sols->Ca_EEP_Clean == 0 || Sols->Ca_EEP_Clean != Sols->Ca_EEP_Clean)
//					{
//						Ca_mpH = ((pCa_Rinse - pCa_Cal_1)*(Ca_mV_Rinse[i] - Ca_mV_Cal_2[i]) - (pCa_Rinse - pCa_Cal_2)*(Ca_mV_Rinse[i] - Ca_mV_Cal_1[i])) / ((pH_TCor_Rinse - pH_TCor_Cal_2)*(pCa_Rinse - pCa_Cal_1) - (pCa_Rinse - pCa_Cal_2)*(pH_TCor_Rinse - pH_TCor_Cal_1));
//						Ca_Slope_CalT[i] = (-(pH_TCor_Rinse - pH_TCor_Cal_1)*(Ca_mV_Rinse[i] - Ca_mV_Cal_2[i]) + (pH_TCor_Rinse - pH_TCor_Cal_2)*(Ca_mV_Rinse[i] - Ca_mV_Cal_1[i])) / ((pH_TCor_Rinse - pH_TCor_Cal_2)*(pCa_Rinse - pCa_Cal_1) - (pCa_Rinse - pCa_Cal_2)*(pH_TCor_Rinse - pH_TCor_Cal_1));
//						Ca_Int = Ca_mV_Cal_2[i] - (Ca_Slope_CalT[i] * pCa_Cal_2);
//					}
//					else	// If there is hardness in clean use that instead of Cal 1
//					{
//						Ca_mpH = ((pCa_Rinse - pCa_Clean)*(Ca_mV_Rinse[i] - Ca_mV_Cal_2[i]) - (pCa_Rinse - pCa_Cal_2)*(Ca_mV_Rinse[i] - Ca_mV_Clean[i])) / ((pH_TCor_Rinse - pH_TCor_Cal_2)*(pCa_Rinse - pCa_Clean) - (pCa_Rinse - pCa_Cal_2)*(pH_TCor_Rinse - pH_TCor_Clean));
//						Ca_Slope_CalT[i] = (-(pH_TCor_Rinse - pH_TCor_Clean)*(Ca_mV_Rinse[i] - Ca_mV_Cal_2[i]) + (pH_TCor_Rinse - pH_TCor_Cal_2)*(Ca_mV_Rinse[i] - Ca_mV_Clean[i])) / ((pH_TCor_Rinse - pH_TCor_Cal_2)*(pCa_Rinse - pCa_Clean) - (pCa_Rinse - pCa_Cal_2)*(pH_TCor_Rinse - pH_TCor_Clean));
//						Ca_Int = Ca_mV_Cal_2[i] - (Ca_Slope_CalT[i] * pCa_Cal_2);
//					}
//
//					if(Ca_mpH > 0)
//					{
//						DEBUG_PRINT(UARTprintf("Setting Ca pH Slope to 0!\n");)
//						Ca_mpH = 0;
//					}
//					else if(Ca_mpH < -3)
//					{
//						DEBUG_PRINT(UARTprintf("Setting Ca pH Slope to -3!\n");)
//						Ca_mpH = -3;
//					}
//					MemoryWrite(Cal_page, OFFSET_CA_1_LOG_K + (i * 4), 4, (uint8_t *) &Ca_mpH);
//					MemoryWrite(Cal_page, OFFSET_ISE_1_INT + ((i + ISEs.Ca.index) * 4), 4, (uint8_t *) &Ca_Int);
//#else	// LINEAR_PH_CORR
//
//#ifndef CALIBRATE_CA_1_R
//					if(Sols->pH_EEP_Cal_2 < 9  || Sols->Ca_EEP_Rinse > 140) // This is Cal 3 or hard rinse, not Cal 2
//					{
//						Ca_Slope_CalT[i] = (Ca_mV_Cal_2[i] - Ca_mV_Rinse[i]) / (pCa_Cal_2 - pCa_Rinse);	// Calibrated slope
//						float Ca_Int = Ca_mV_Cal_2[i] - (Ca_Slope_CalT[i] * pCa_Cal_2);
//						MemoryWrite(Cal_page, OFFSET_ISE_1_INT + ((i + ISEs.Ca.index) * 4), 4, (uint8_t *) &Ca_Int);
//
//
//#ifdef PH_LOG_K
//						if(Sols->pH_EEP_Cal_2 < 9 && Sols->Ca_EEP_Cal_1 != 0)	// This is Cal 3/Cal 4 setup
//						{
//							float Log_K_Ca_pH = log10(0.4833 * pow((Ca_mV_Cal_1[i] - Ca_mV_Rinse[i]), 2) + 11.603 * (Ca_mV_Cal_1[i] - Ca_mV_Rinse[i]) + 1.201);//1.6;	// Calculate a Log k
//							MemoryWrite(Cal_page, OFFSET_CA_1_LOG_K + (i * 4), 4, (uint8_t *) &Log_K_Ca_pH);
//						}
//#endif
//					}
//					else
//					{
//						Ca_Slope_CalT[i] = (Ca_mV_Cal_2[i] - Ca_mV_Cal_1[i]) / (pCa_Cal_2 - pCa_Cal_1);	// Calibrated slope
//						float Ca_Int = Ca_mV_Cal_1[i] - (Ca_Slope_CalT[i] * pCa_Cal_1);
//						MemoryWrite(Cal_page, OFFSET_ISE_1_INT + ((i + ISEs.Ca.index) * 4), 4, (uint8_t *) &Ca_Int);
//					}
////					Ca_Int[i] = Ca_mV_Cal_1[i] - (Ca_Slope_CalT[i] * pCa_Cal_1);
//#else
//					Ca_Slope_CalT[i] = (Ca_mV_Rinse[i] - Ca_mV_Cal_1[i]) / (pCa_Rinse - pCa_Cal_1);	// Calibrated slope
//					Ca_Int[i] = Ca_mV_Cal_1[i] - (Ca_Slope_CalT[i] * pCa_Cal_1);
//					MemoryWrite(Cal_page, OFFSET_CA_1_MV_CAL_2, 4, (uint8_t *) &Ca_mV_Cal_2[0]);
//					MemoryWrite(Cal_page, OFFSET_CA_2_MV_CAL_2, 4, (uint8_t *) &Ca_mV_Cal_2[1]);
//#endif	// CALIBRATE_CA_1_R
//#endif	// LINEAR_PH_CORR
//					if(Ca_Slope_CalT[i] <= CA_SLOPE_HIGH && Ca_Slope_CalT[i] >= CA_SLOPE_LOW)
//					{
//						Ca_Cal_Check[i] = 1;
//						L_Ca_Chosen = i;
//						App_Status |= (1 << APP_CALCIUM);
//					}
//				}
//
//				//
//				// Magnesium and Total Hardness Calibration
//				//
//				float pTH_Cal_1, pTH_Cal_2, pTH_Clean;
//
//				// Values are concentration, calculate p-values
//				pTH_Cal_1 = Calc_pTH(Sols->Ca_EEP_Cal_1, Sols->TH_EEP_Cal_1, log_K_Ca_Mg, T_Cal, Sols->IS_CAL_1);
//				pTH_Cal_2 = Calc_pTH(Sols->Ca_EEP_Cal_2, Sols->TH_EEP_Cal_2, log_K_Ca_Mg, T_Cal, Sols->IS_CAL_2);
//				pTH_Clean = Calc_pTH(Sols->Ca_EEP_Clean, Sols->TH_EEP_Clean, log_K_Ca_Mg, T_Cal, Sols->IS_CLEAN);
//
//				for(i = 0; i < ISEs.TH.size; i++)
//				{
//#ifndef CALIBRATE_TH_R_2
//					TH_Slope_CalT[i] = (TH_mV_Cal_2[i] - TH_mV_Cal_1[i]) / (pTH_Cal_2 - pTH_Cal_1); // Calibrated slope
//					TH_Int[i] = TH_mV_Cal_1[i] - (TH_Slope_CalT[i] * pTH_Cal_1);
//#else
//#ifdef LINEAR_PH_CORR
//					float TH_mpH = -2, TH_Int;
//					if(Sols->TH_EEP_Clean == 0 || Sols->TH_EEP_Clean != Sols->TH_EEP_Clean)
//					{
//						TH_Slope_CalT[i] = (TH_mV_Cal_2[i] - TH_mV_Rinse[i] - (TH_mpH * (pH_TCor_Cal_2 - pH_TCor_Rinse))) / (pTH_Cal_2 - pTH_Rinse); // Calibrated slope
//						TH_Int = TH_mV_Cal_2[i] - (TH_Slope_CalT[i] * pTH_Cal_2);
//					}
//					else	// If there is hardness in Clean use that rather than Cal 1
//					{
//						TH_mpH = ((pTH_Rinse - pTH_Clean)*(TH_mV_Rinse[i] - TH_mV_Cal_2[i]) - (pTH_Rinse - pTH_Cal_2)*(TH_mV_Rinse[i] - TH_mV_Clean[i])) / ((pH_TCor_Rinse - pH_TCor_Cal_2)*(pTH_Rinse - pTH_Clean) - (pTH_Rinse - pTH_Cal_2)*(pH_TCor_Rinse - pH_TCor_Clean));
//						TH_Slope_CalT[i] = (-(pH_TCor_Rinse - pH_TCor_Clean)*(TH_mV_Rinse[i] - TH_mV_Cal_2[i]) + (pH_TCor_Rinse - pH_TCor_Cal_2)*(TH_mV_Rinse[i] - TH_mV_Clean[i])) / ((pH_TCor_Rinse - pH_TCor_Cal_2)*(pTH_Rinse - pTH_Clean) - (pTH_Rinse - pTH_Cal_2)*(pH_TCor_Rinse - pH_TCor_Clean));
//						TH_Int = TH_mV_Cal_2[i] - (TH_Slope_CalT[i] * pTH_Cal_2);
//					}
//					MemoryWrite(Cal_page, OFFSET_MG_1_PH_SLOPE + (i * 4), 4, (uint8_t *) &TH_mpH);
//					MemoryWrite(Cal_page, OFFSET_ISE_1_INT + ((i + ISEs.TH.index) * 4), 4, (uint8_t *) &TH_Int);
//
//#else	// LINEAR_PH_CORR
//					TH_Slope_CalT[i] = (TH_mV_Cal_2[i] - TH_mV_Rinse[i]) / (pTH_Cal_2 - pTH_Rinse); // Calibrated slope
//					float TH_Int = TH_mV_Cal_2[i] - (TH_Slope_CalT[i] * pTH_Cal_2);
//					MemoryWrite(Cal_page, OFFSET_ISE_1_INT + ((i + ISEs.TH.index) * 4), 4, (uint8_t *) &TH_Int);
//
//#endif	// LINEAR_PH_CORR
//
////					TH_Int[i] = TH_mV_Cal_2[i] - (TH_Slope_CalT[i] * pTH_Cal_2);
////					MemoryWrite(Cal_page, OFFSET_MG_1_MV_CAL_1, 4, (uint8_t *) &TH_mV_Cal_1[0]);
////					MemoryWrite(Cal_page, OFFSET_MG_2_MV_CAL_1, 4, (uint8_t *) &TH_mV_Cal_1[1]);
//
//#ifdef PH_LOG_K
//					if(Sols->pH_EEP_Cal_2 < 9 && Sols->TH_EEP_Cal_1 != 0)	// This is Cal 3/Cal 4 setup
//					{
//						float Log_K_TH_pH = log10(0.6456 * pow((TH_mV_Cal_1[i] - TH_mV_Rinse[i]), 2) + 24.499 * (TH_mV_Cal_1[i] - TH_mV_Rinse[i]) +1.1208); //2.1;	// Calculate a Log k
//						MemoryWrite(Cal_page, OFFSET_TH_1_LOG_K + (i * 4), 4, (uint8_t *) &Log_K_TH_pH);
//					}
//#endif	// PH_LOG_K
//#endif	// #ifndef CALIBRATE_TH_R_2
//					if(TH_Slope_CalT[i] <= TH_SLOPE_HIGH && TH_Slope_CalT[i] >= TH_SLOPE_LOW)
//					{
//						TH_Cal_Check[i] = 1;
//						L_TH_Chosen = i;
//						App_Status |= (1 << APP_MAGNESIUM);
//					}
//				}
//
//#ifdef TH_ITERATED_MATH
//				// Calculate assuming Mg sensor (Nick's method)
//				float pMg_Rinse, pMg_Cal_2;//, pMg_Cal_1;
//				float Mg_Slope_CalT[2];
//				if(ISEs.TH.size > 0)
//				{
//					pMg_Rinse = Calc_pMg(Sols->Ca_EEP_Rinse, Sols->TH_EEP_Rinse, T_Cal, Sols->IS_RINSE);
////					pMg_Cal_1 = Calc_pMg(Ca_EEP_Cal_1, TH_EEP_Cal_1, T_Cal, IS_CAL_1);
////					pMg_Cal_1 = Calc_pMg(Sols->Ca_EEP_Cal_1, Sols->TH_EEP_Cal_1, T_Cal, Sols->IS_CAL_1);
//					pMg_Cal_2 = Calc_pMg(Sols->Ca_EEP_Cal_2, Sols->TH_EEP_Cal_2, T_Cal, Sols->IS_CAL_2);
//
//					float /*Mg_Int[2],*/ log_K_Ca_Mg_Nick[2];
//					for(i = 0; i < 2; i++)
//					{
//						Mg_Slope_CalT[i] = (TH_mV_Cal_2[i] - TH_mV_Rinse[i]) / (pMg_Cal_2 - pMg_Rinse); // Calibrated slope
////						Mg_Int[i] = TH_mV_Rinse[i] - (Mg_Slope_CalT[i] * pMg_Rinse);
//						//					if(Mg_Slope_CalT[i] <= TH_SLOPE_HIGH && Mg_Slope_CalT[i] >= TH_SLOPE_LOW)
//						//					{
//						//						TH_Cal_Check[i] = 1;
//						//						L_TH_Chosen = i;
//						//					}
//
//						log_K_Ca_Mg_Nick[i] = -0.5;//0.0005 * pow((TH_mV_Cal_1[i] - TH_mV_Rinse[i]), 3) - 0.0158 * pow((TH_mV_Cal_1[i] - TH_mV_Rinse[i]), 2) + 0.2415 * (TH_mV_Cal_1[i] - TH_mV_Rinse[i]) - 1.5766;
//					}
//
//
//					for(i = 0; i < 2; i++)
//					{
//						MemoryWrite(Cal_page, OFFSET_MG_1_SLOPE + (i * 4), 4, (uint8_t *) &Mg_Slope_CalT[i]);
////						MemoryWrite(Cal_page, OFFSET_MG_1_INT + (i * 4), 4, (uint8_t *) &Mg_Int[i]);
//						MemoryWrite(Cal_page, OFFSET_MG_1_LOG_K + (i * 4), 4, (uint8_t *) &log_K_Ca_Mg_Nick[i]);
//					}
//				}
//#endif
//
//				//
//				// NH4 Calibration
//				//
//				float pNH4_Cal_1, pNH4_Clean, pNH4_Cal_2;
//
//				// Values are concentration, calculate p-values
//				pNH4_Cal_1 = Calc_pNH4(Sols->NH4_EEP_Cal_1, pH_TCor_Cal_1, SM_NA_CAL_1, T_Cal, Sols->IS_CAL_1);
//				pNH4_Clean = Calc_pNH4(Sols->NH4_EEP_Clean, pH_TCor_Clean, 0, T_Cal, Sols->IS_CLEAN);
//				pNH4_Cal_2 = Calc_pNH4(Sols->NH4_EEP_Cal_2, pH_TCor_Cal_2, 0, T_Cal, Sols->IS_CAL_2);
//
//				for(i = 0; i < ISEs.NH4.size; i++)
//				{
//					if(Sols->pH_EEP_Clean > 8.5)	// pH 9 Clean, calibrate between Cal 1 and Rinse
//					{
//						float NH4_Int;
//						if(Sols->NH4_EEP_Cal_2 != 0)	// This is pH 9 Clean and Cal 5
//						{
//#ifdef NH4_PH_CORR
//							float NH4_mpH = ((pNH4_Rinse - pNH4_Cal_1) * (NH4_mV_Rinse[i] - NH4_mV_Cal_2[i]) - (pNH4_Rinse - pNH4_Cal_2) * (NH4_mV_Rinse[i] - NH4_mV_Cal_1[i])) / ((pH_TCor_Rinse - pH_TCor_Cal_2) * (pNH4_Rinse - pNH4_Cal_1) - (pNH4_Rinse - pNH4_Cal_2) * (pH_TCor_Rinse - pH_TCor_Cal_1));
//							NH4_Slope_CalT[i] = (-(pH_TCor_Rinse - pH_TCor_Cal_1) * (NH4_mV_Rinse[i] - NH4_mV_Cal_2[i]) + (pH_TCor_Rinse - pH_TCor_Cal_2) * (NH4_mV_Rinse[i] - NH4_mV_Cal_1[i])) / ((pH_TCor_Rinse - pH_TCor_Cal_2) * (pNH4_Rinse - pNH4_Cal_1) - (pNH4_Rinse - pNH4_Cal_2) * (pH_TCor_Rinse - pH_TCor_Cal_1));
//							NH4_Int = NH4_mV_Cal_2[i] - (NH4_Slope_CalT[i] * pNH4_Cal_2);
//							MemoryWrite(Cal_page, OFFSET_NH4_1_LOG_K + ((i) * 4), 4, (uint8_t *) &NH4_mpH);
//
//#else
//							NH4_Slope_CalT[i] = (NH4_mV_Rinse[i] - NH4_mV_Cal_2[i]) / (pNH4_Rinse - pNH4_Cal_2);
//							NH4_Int = NH4_mV_Cal_2[i] - (NH4_Slope_CalT[i] * pNH4_Cal_2);
//#endif
//						}
//						else
//						{
//							NH4_Slope_CalT[i] = (NH4_mV_Rinse[i] - NH4_mV_Cal_1[i]) / (pNH4_Rinse - pNH4_Cal_1);
//							NH4_Int = NH4_mV_Cal_1[i] - (NH4_Slope_CalT[i] * pNH4_Cal_1);
//						}
//
//						MemoryWrite(Cal_page, OFFSET_ISE_1_INT + ((i + ISEs.NH4.index) * 4), 4, (uint8_t *) &NH4_Int);
//					}
//					else if(Sols->pH_EEP_Cal_2 < 9 && Sols->NH4_EEP_Cal_1 == 0)	// This is Cal 3
//					{
//						NH4_Slope_CalT[i] = (NH4_mV_Clean[i] - NH4_mV_Cal_2[i]) / (pNH4_Clean - pNH4_Cal_2);
//						float NH4_Int = NH4_mV_Cal_2[i] - (NH4_Slope_CalT[i] * pNH4_Cal_2);
//						MemoryWrite(Cal_page, OFFSET_ISE_1_INT + ((i + ISEs.NH4.index) * 4), 4, (uint8_t *) &NH4_Int);
//
////#ifdef PH_LOG_K
////						if(Sols->pH_EEP_Cal_2 < 9 && Sols->NH4_EEP_Cal_1 != 0)	// This is Cal 3/Cal 4 setup
////						{
////							float Log_K_NH4_pH = -1;	// TOD: Calculate a Log k
////							MemoryWrite(Cal_page, OFFSET_NH4_1_LOG_K + (i * 4), 4, (uint8_t *) &Log_K_NH4_pH);
////						}
////#endif	// PH_LOG_K
//					}
//					else
//					{
//						NH4_Slope_CalT[i] = (NH4_mV_Clean[i] - NH4_mV_Cal_1[i]) / (pNH4_Clean - pNH4_Cal_1);
//						float NH4_Int = NH4_mV_Cal_1[i] - (NH4_Slope_CalT[i] * pNH4_Cal_1);
//						MemoryWrite(Cal_page, OFFSET_ISE_1_INT + ((i + ISEs.NH4.index) * 4), 4, (uint8_t *) &NH4_Int);
//					}
////					NH4_Int[i] = NH4_mV_Cal_1[i] - (NH4_Slope_CalT[i] * pNH4_Cal_1);
//					if(NH4_Slope_CalT[i] <= NH4_SLOPE_HIGH && NH4_Slope_CalT[i] >= NH4_SLOPE_LOW)
//					{
//						NH4_Cal_Check[i] = 1;
//						L_NH4_Chosen = i;
//						App_Status |= (1 << APP_AMMONIUM);
//					}
//				}
//
//
//
//				//
//				// Conductivity Calibration
//				//
//				// Adjust conductivity to use a fixed point at 61.8 of 270001.09 if factory cal wasn't read successfully
//				float CondLow;
//				float CalConductivityV1LowInv = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_COND_READ_LOW_POINT, 4));
//				if(CalConductivityV1LowInv == CalConductivityV1LowInv)
//				{
//					CondLow = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_COND_LOW_POINT_CAL, 4));
//					if(CalConductivityV1LowInv > 1)	// 8/9/2023: Factory Cal was done recording the mV difference, current adjust reading for new calibration
//						CalConductivityV1LowInv = ((10.76 * 0.795) * 1000000) / CalConductivityV1LowInv;
//					else
//						CalConductivityV1LowInv *= 1000000;
//				}
//				else
//				{
//					CalConductivityV1LowInv = ((10.76 * 0.795) * 1000000) / (270001.09 * 2);
//					CondLow = 61.8;
//				}
//
//				float I_Low, I_Mid, I_High;
//
//				// Read the currents off the memory, these should be saved during the QC process
//				EEPROMRead((uint32_t *) &I_Low, OFFSET_COND_I_LOW, 4);
//				EEPROMRead((uint32_t *) &I_Mid, OFFSET_COND_I_MID, 4);
//				EEPROMRead((uint32_t *) &I_High, OFFSET_COND_I_HIGH, 4);
//
//				if(I_Low != I_Low)
//					I_Low = 10.76 * 0.795;	// Average from circuits before ARV1_0B
//				if(I_Mid != I_Mid)
//					I_Mid = 19.89 * 0.8;	// Average from circuits before ARV1_0B
//				if(I_High != I_High)
//					I_High = 43.57 * .812;	// Average from circuits before ARV1_0B
//
//				// Multiply the currents by 1,000,000 before dividing to avoid small numbers, in reality it's to convert our calibrants from uS to S but the computer math is better done this way
//				float CalConductivityV2LowInv = (1000000 * I_Low) / CalConductivityV2Low;
//				float CalConductivityV1MidInv = (1000000 * I_Mid) / CalConductivityV1Mid;
//				float CalConductivityV2MidInv = (1000000 * I_Mid) / CalConductivityV2Mid;
//				float CalConductivityV2HighInv = (1000000 * I_High) / CalConductivityV2High;
//				float CalConductivityV3HighInv = (1000000 * I_High) / CalConductivityV3High;
////				float CalConductivityDeltaVLow = CalConductivityV2LowInv - CalConductivityV1LowInv;
////				float CalConductivityDeltaVMid = CalConductivityV2MidInv - CalConductivityV1MidInv;
////				float CalConductivityDeltaVHigh = CalConductivityV3HighInv - CalConductivityV2HighInv;
//
//				float CalConductivitySlopeLow, CalConductivitySlopeMid, CalConductivitySlopeHigh, CalConductivityKLow, CalConductivityKMid, CalConductivityKHigh;
//				if(1)	// This is Cal 3, Cal 3 is highest conductivity, Cal 1 is lowest conductivity
//				{
//					// Put the 3 calibrants used in the array twice each, order doesn't matter here because the array will be sorted from smallest to largest
//					// Really only need 5 points with the highest conductivity calibrant in the array once, but to make it universal have an extra spot and the last spot will be ignored after sorting
//					float CalConds[3] = {Sols->Cond_EEP_Clean*(1 + Sols->Clean_Cond_TComp*(T_Cal - 25)), Sols->Cond_EEP_Cal_1*(1 + Sols->Cal_1_Cond_TComp*(T_Cal - 25)), Sols->Cond_EEP_Cal_2*(1 + Sols->Cal_2_Cond_TComp*(T_Cal - 25))};
//					SortArray(CalConds, 3);
//
////					float CalConductivityDeltaCLow = (CalConds[0] - CondLow);
////					float CalConductivityDeltaCMid = (CalConds[1] - CalConds[0]);
////					float CalConductivityDeltaCHigh = (CalConds[2] - CalConds[1]);
//
//					CalConductivitySlopeLow =  (CalConductivityV2LowInv - CalConductivityV1LowInv) / (CalConds[0] - CondLow);
//					CalConductivitySlopeMid = (CalConductivityV2MidInv - CalConductivityV1MidInv) / (CalConds[1] - CalConds[0]);
//					CalConductivitySlopeHigh = (CalConductivityV3HighInv - CalConductivityV2HighInv) / (CalConds[2] - CalConds[1]);
//					CalConductivityKLow = I_Low / CalConductivityV2Low - CalConductivitySlopeLow * CalConds[0]/1000000;
//					CalConductivityKMid = I_Mid / CalConductivityV2Mid - CalConductivitySlopeMid * CalConds[1]/1000000;
//					CalConductivityKHigh = I_High / CalConductivityV3High - CalConductivitySlopeHigh * CalConds[2]/1000000;
//				}
//
//				if(CalConductivitySlopeLow >= COND_SLOPE_1_LOW && CalConductivitySlopeLow <= COND_SLOPE_1_HIGH
//						&& CalConductivitySlopeMid >= COND_SLOPE_2_LOW && CalConductivitySlopeMid <= COND_SLOPE_2_HIGH
//						&& CalConductivitySlopeHigh >= COND_SLOPE_3_LOW && CalConductivitySlopeHigh <= COND_SLOPE_3_HIGH)
//				{
//					App_Status |= (1 << APP_COND);
//				}
//
//				// Mark if all sensors of a type failed calibration
//				uint8_t Cal_failed = 0; // 0, 0, 0, 0, NH4, TH, Ca, pH
//
//				// Determine if conductivity passed calibration, if it did not set error flag
//				if(CalConductivitySlopeLow < COND_SLOPE_1_LOW || CalConductivitySlopeLow > COND_SLOPE_1_HIGH
//						|| CalConductivitySlopeMid < COND_SLOPE_2_LOW || CalConductivitySlopeMid > COND_SLOPE_2_HIGH
//						|| CalConductivitySlopeHigh < COND_SLOPE_3_LOW || CalConductivitySlopeHigh > COND_SLOPE_3_HIGH)
//				{
//					Cal_failed++;
//					gui32Error |= CAL_FAILED_COND_SLOPES;
//				}
//				else
//				{
//					Calibration_Status |= 1 << 11;
//					App_Status |= (1 << APP_COND);
//				}
//
//
//				if(ISEs.pH_H2.size > 0)
//				{
//					uint8_t pH_H2_passed = 0;
//					for(i = 0; i < ISEs.pH_H2.size; i++)
//						pH_H2_passed += pH_H2_Cal_Check[i];
//
//					if(pH_H2_passed == 0)
//					{
//						Cal_failed++;
//						gui32Error |= CAL_FAILED_PH_H2_SLOPES;
//					}
//				}
//
//				if(ISEs.pH_Cr.size > 0)
//				{
//					uint8_t pH_Cr_passed = 0;
//					for(i = 0; i < ISEs.pH_Cr.size; i++)
//						pH_Cr_passed += pH_Cr_Cal_Check[i];
//
//					if(pH_Cr_passed == 0)
//					{
//						Cal_failed++;
//						gui32Error |= CAL_FAILED_PH_CR_SLOPES;
//					}
//				}
//
//				if(ISEs.TH.size > 0)
//				{
//					uint8_t TH_passed = 0;
//					for(i = 0; i < ISEs.TH.size; i++)
//						TH_passed += TH_Cal_Check[i];
//
//					if(TH_passed == 0)
//					{
//						Cal_failed++;
//						gui32Error |= CAL_FAILED_TH_SLOPES;
//					}
//				}
//
//				if(ISEs.NH4.size > 0)
//				{
//					uint8_t NH4_passed = 0;
//					for(i = 0; i < ISEs.NH4.size; i++)
//						NH4_passed += NH4_Cal_Check[i];
//
//					if(NH4_passed == 0)
//					{
//						Cal_failed++;
//						gui32Error |= CAL_FAILED_NH4_SLOPES;
//					}
//				}
//
//				if(ISEs.Ca.size > 0)
//				{
//					uint8_t Ca_passed = 0;
//					for(i = 0; i < ISEs.Ca.size; i++)
//						Ca_passed += Ca_Cal_Check[i];
//
//					if(Ca_passed == 0)
//					{
//						Cal_failed++;
//						gui32Error |= CAL_FAILED_CA_SLOPES;
//					}
//				}
//
//				// Choose sensors in calibration
//				uint8_t L_Chosen_Sensors = 0;
//				if(Sols->pH_EEP_Cal_2 < 9 && Sols->pH_EEP_Cal_2 > 7)	// This is Cal 3, can't calculate linearity because we only have 2 points
//				{
//					//
//					// Choose sensors based on closest to theoretical slope
//					//
//					for(i = 0; i < ISEs.pH_H2.size; i++)
//						if(abs_val((pH_H2_Slope_CalT[i] * (25 + 273)/(T_Cal + 273)) - (-59.9)) < abs_val((pH_H2_Slope_CalT[L_pH_H2_Chosen] * (25 + 273)/(T_Cal + 273)) - (-59.9)) && pH_H2_Cal_Check[i] == 1)
//							L_pH_H2_Chosen = i;
//
//					for(i = 0; i < ISEs.pH_Cr.size; i++)
//						if(abs_val((pH_Cr_Slope_CalT[i] * (25 + 273)/(T_Cal + 273)) - (-59.9)) < abs_val((pH_Cr_Slope_CalT[L_pH_Cr_Chosen] * (25 + 273)/(T_Cal + 273)) - (-59.9)) && pH_Cr_Cal_Check[i] == 1)
//							L_pH_Cr_Chosen = i;
//
//					for(i = 0; i < ISEs.TH.size; i++)
//						if(abs_val((TH_Slope_CalT[i] * (25 + 273)/(T_Cal + 273)) - (-29.5)) < abs_val((TH_Slope_CalT[L_TH_Chosen] * (25 + 273)/(T_Cal + 273)) - (-29.5)) && TH_Cal_Check[i] == 1)
//							L_TH_Chosen = i;
//
//					for(i = 0; i < ISEs.NH4.size; i++)
//						if(abs_val((NH4_Slope_CalT[i] * (25 + 273)/(T_Cal + 273)) - (-59.9)) < abs_val((NH4_Slope_CalT[L_NH4_Chosen] * (25 + 273)/(T_Cal + 273)) - (-59.9)) && NH4_Cal_Check[i] == 1)
//							L_NH4_Chosen = i;
//
//					for(i = 0; i < ISEs.Ca.size; i++)
//						if(abs_val((Ca_Slope_CalT[i] * (25 + 273)/(T_Cal + 273)) - (-29.5)) < abs_val((Ca_Slope_CalT[L_Ca_Chosen] * (25 + 273)/(T_Cal + 273)) - (-29.5)) && Ca_Cal_Check[i] == 1)
//							L_Ca_Chosen = i;
//				}
//				else if(Sols->pH_EEP_Cal_2 < 7)	// This is Cal 5/pH 9 Clean/Cal 6 setup
//				{
//					//
//					// Choose sensors based on closest to theoretical slope
//					//
//					for(i = 0; i < ISEs.pH_H2.size; i++)
//						if(abs_val((pH_H2_Slope_CalT[i] * (25 + 273)/(T_Cal + 273)) - (-59.9)) < abs_val((pH_H2_Slope_CalT[L_pH_H2_Chosen] * (25 + 273)/(T_Cal + 273)) - (-59.9)) && pH_H2_Cal_Check[i] == 1)
//							L_pH_H2_Chosen = i;
//
//					//				float pH_r2[3], Ca_r2[2], TH_r2[2], NH4_r2[3];
//					float ISE_r2[10] = {0,0,0,0,0,0,0,0,0,0};
//					//				float *pH_H2_r2 = &ISE_r2[ISEs.pH_H2.index];
//					float *pH_Cr_r2 = &ISE_r2[ISEs.pH_Cr.index];
//					float *TH_r2 = &ISE_r2[ISEs.TH.index];
//					float *NH4_r2 = &ISE_r2[ISEs.NH4.index];
//					float *Ca_r2 = &ISE_r2[ISEs.Ca.index];
//
//	//#ifdef TH_ITERATED_MATH
//	//				float pMg_Rinse;
//	//				pMg_Rinse = Calc_pMg(Ca_EEP_Rinse, TH_EEP_Rinse, T_Cal, IS_RINSE);
//	//#endif
//
//					// Calculate the R^2 value for each line to determine which is the most linear for each sensor type
//					for(i = 0; i < ISEs.pH_Cr.size; i++)
//					{
//						float pH_xs[3] = {pH_TCor_Rinse, pH_TCor_Cal_1, pH_TCor_Clean};
//						float pH_ys[3] = {pH_Cr_mV_Rinse[i], pH_Cr_mV_Cal_1[i], pH_Cr_mV_Clean[i]};
//						pH_Cr_r2[i] = RSQ(pH_xs, pH_ys, 3);
//					}
//					for(i = 0; i < ISEs.Ca.size; i++)
//					{
//						float Ca_xs[3] = {pCa_Rinse, pCa_Clean, pCa_Cal_2};
//						float Ca_ys[3] = {Ca_mV_Rinse[i], Ca_mV_Clean[i], Ca_mV_Cal_2[i]};
//						Ca_r2[i] = RSQ(Ca_xs, Ca_ys, 3);
//					}
//					for(i = 0; i < ISEs.TH.size; i++)
//					{
//						float TH_xs[3] = {pTH_Rinse, pTH_Clean, pTH_Cal_2};
//						float TH_ys[3] = {TH_mV_Rinse[i], TH_mV_Clean[i], TH_mV_Cal_2[i]};
//						TH_r2[i] = RSQ(TH_xs, TH_ys, 3);
//					}
//					for(i = 0; i < ISEs.NH4.size; i++)
//					{
//						float NH4_xs[3] = {pNH4_Rinse, pNH4_Cal_1, pNH4_Cal_2};
//						float NH4_ys[3] = {NH4_mV_Rinse[i], NH4_mV_Cal_1[i], NH4_mV_Cal_2[i]};
//						NH4_r2[i] = RSQ(NH4_xs, NH4_ys, 3);
//					}
//
//					DEBUG_PRINT(UARTprintf("R Squared for each line using Prerinse:\n");)
//					for(i = 0; i < ISEs.pH_Cr.size; i++)
//						{DEBUG_PRINT(UARTprintf("pH %d: %d\n", i + 1, (int) (pH_Cr_r2[i] * 1000));)}
//					for(i = 0; i < ISEs.TH.size; i++)
//						{DEBUG_PRINT(UARTprintf("TH %d: %d\n", i + 1, (int) (TH_r2[i] * 1000));)}
//					for(i = 0; i < ISEs.NH4.size; i++)
//						{DEBUG_PRINT(UARTprintf("NH4 %d: %d\n", i + 1, (int) (NH4_r2[i] * 1000));)}
//					for(i = 0; i < ISEs.Ca.size; i++)
//						{DEBUG_PRINT(UARTprintf("Ca %d: %d\n", i + 1, (int) (Ca_r2[i] * 1000));)}
//
//					//
//					// Choose sensors based on linearity
//					//
//					for(i = 0; i < ISEs.pH_Cr.size; i++)
//						if(pH_Cr_r2[i] > pH_Cr_r2[L_pH_Cr_Chosen] && pH_Cr_Cal_Check[i] == 1)
//							L_pH_Cr_Chosen = i;
//
//					for(i = 0; i < ISEs.TH.size; i++)
//						if(TH_r2[i] > TH_r2[L_TH_Chosen] && TH_Cal_Check[i] == 1)
//							L_TH_Chosen = i;
//
//					for(i = 0; i < ISEs.NH4.size; i++)
//						if(NH4_r2[i] > NH4_r2[L_NH4_Chosen] && NH4_Cal_Check[i] == 1)
//							L_NH4_Chosen = i;
//
//					for(i = 0; i < ISEs.Ca.size; i++)
//						if(Ca_r2[i] > Ca_r2[L_Ca_Chosen] && Ca_Cal_Check[i] == 1)
//							L_Ca_Chosen = i;
//				}
//				else
//				{
//					//				float pH_r2[3], Ca_r2[2], TH_r2[2], NH4_r2[3];
//					float ISE_r2[10] = {0,0,0,0,0,0,0,0,0,0};
//					//				float *pH_H2_r2 = &ISE_r2[ISEs.pH_H2.index];
//					float *pH_Cr_r2 = &ISE_r2[ISEs.pH_Cr.index];
//					float *TH_r2 = &ISE_r2[ISEs.TH.index];
//					float *NH4_r2 = &ISE_r2[ISEs.NH4.index];
//					float *Ca_r2 = &ISE_r2[ISEs.Ca.index];
//
//	//#ifdef TH_ITERATED_MATH
//	//				float pMg_Rinse;
//	//				pMg_Rinse = Calc_pMg(Ca_EEP_Rinse, TH_EEP_Rinse, T_Cal, IS_RINSE);
//	//#endif
//
//					// Calculate the R^2 value for each line to determine which is the most linear for each sensor type
//					for(i = 0; i < ISEs.pH_Cr.size; i++)
//					{
//						float pH_xs[3] = {pH_TCor_Rinse, pH_TCor_Cal_1, pH_TCor_Cal_2};
//						float pH_ys[3] = {pH_Cr_mV_Rinse[i], pH_Cr_mV_Cal_1[i], pH_Cr_mV_Cal_2[i]};
//						pH_Cr_r2[i] = RSQ(pH_xs, pH_ys, 3);
//					}
//					for(i = 0; i < ISEs.Ca.size; i++)
//					{
//						float Ca_xs[3] = {pCa_Rinse, pCa_Cal_1, pCa_Cal_2};
//						float Ca_ys[3] = {Ca_mV_Rinse[i], Ca_mV_Cal_1[i], Ca_mV_Cal_2[i]};
//						Ca_r2[i] = RSQ(Ca_xs, Ca_ys, 3);
//					}
//					for(i = 0; i < ISEs.TH.size; i++)
//					{
//						float TH_xs[3] = {pTH_Rinse, pTH_Cal_1, pTH_Cal_2};
//						float TH_ys[3] = {TH_mV_Rinse[i], TH_mV_Cal_1[i], TH_mV_Cal_2[i]};
//						TH_r2[i] = RSQ(TH_xs, TH_ys, 3);
//					}
//					for(i = 0; i < ISEs.NH4.size; i++)
//					{
//						float NH4_xs[3] = {pNH4_Rinse, pNH4_Cal_1, pNH4_Clean};
//						float NH4_ys[3] = {NH4_mV_Rinse[i], NH4_mV_Cal_1[i], NH4_mV_Cal_2[i]};
//						NH4_r2[i] = RSQ(NH4_xs, NH4_ys, 3);
//					}
//
//					DEBUG_PRINT(UARTprintf("R Squared for each line using Prerinse:\n");)
//					for(i = 0; i < ISEs.pH_Cr.size; i++)
//						{DEBUG_PRINT(UARTprintf("pH %d: %d\n", i + 1, (int) (pH_Cr_r2[i] * 1000));)}
//					for(i = 0; i < ISEs.TH.size; i++)
//						{DEBUG_PRINT(UARTprintf("TH %d: %d\n", i + 1, (int) (TH_r2[i] * 1000));)}
//					for(i = 0; i < ISEs.NH4.size; i++)
//						{DEBUG_PRINT(UARTprintf("NH4 %d: %d\n", i + 1, (int) (NH4_r2[i] * 1000));)}
//					for(i = 0; i < ISEs.Ca.size; i++)
//						{DEBUG_PRINT(UARTprintf("Ca %d: %d\n", i + 1, (int) (Ca_r2[i] * 1000));)}
//
//					//
//					// Choose sensors based on linearity
//					//
//					for(i = 0; i < ISEs.pH_Cr.size; i++)
//						if(pH_Cr_r2[i] > pH_Cr_r2[L_pH_Cr_Chosen] && pH_Cr_Cal_Check[i] == 1)
//							L_pH_Cr_Chosen = i;
//
//					for(i = 0; i < ISEs.TH.size; i++)
//						if(TH_r2[i] > TH_r2[L_TH_Chosen] && TH_Cal_Check[i] == 1)
//							L_TH_Chosen = i;
//
//					for(i = 0; i < ISEs.NH4.size; i++)
//						if(NH4_r2[i] > NH4_r2[L_NH4_Chosen] && NH4_Cal_Check[i] == 1)
//							L_NH4_Chosen = i;
//
//					for(i = 0; i < ISEs.Ca.size; i++)
//						if(Ca_r2[i] > Ca_r2[L_Ca_Chosen] && Ca_Cal_Check[i] == 1)
//							L_Ca_Chosen = i;
//				}
//
//				L_Chosen_Sensors =
//						(L_pH_H2_Chosen << ISEs.pH_H2.StorBit) |
//						(L_pH_Cr_Chosen << ISEs.pH_Cr.StorBit) |
//						(L_TH_Chosen << ISEs.TH.StorBit) |
//						(L_NH4_Chosen << ISEs.NH4.StorBit) |
//						(L_Ca_Chosen << ISEs.Ca.StorBit);
//
//				if(Cal_failed == 0)
//				{
//					Calibration_Status |= 1;
//					App_Status |= (1 >> APP_CALIBRATED);
//				}
//
//				for(i = 0; i < 10; i++)
//					Calibration_Status |= (ISE_Cal_Check[i] << i + 1);
//
//				// 8/12/2020: Removed rinse check from calibration status, added re-pump status so I can tell if re-pump occurred and what solution
//				if(calibrants_to_pump == 5 && Cal_Order[4] == V_RINSE)
//					Calibration_Status |= (0x00000001 << 19);
//				else if(calibrants_to_pump == 5 && Cal_Order[4] == V_CAL_1)
//					Calibration_Status |= (0x00000001 << 20);
//				else if(calibrants_to_pump == 5 && Cal_Order[4] == V_CAL_2)
//					Calibration_Status |= (0x00000001 << 21);
//				else if(calibrants_to_pump == 5 && Cal_Order[4] == V_CLEAN)
//					Calibration_Status |= (0x00000001 << 22);
//
////				// EEPROM commands require multiples of 4, so create Device_Serial to be 8 long even though there are only 7 bytes necessary
////				uint8_t Device_Serial[8];
////				EEPROMRead((uint32_t *) Device_Serial, OFFSET_SERIAL_NUMBER, 8);
////				MemoryWrite(Cal_page, OFFSET_CAL_DEVICE_SERIAL, 7, Device_Serial);
//
//				// Save this cal number if sensors passed calibration
//				for(i = 0; i < 10; i++)
//					if(ISE_Cal_Check[i])
//						MemoryWrite(Cal_page, OFFSET_PH_1_LAST_P_CAL + i, 1, (uint8_t *) &Cal_Number);
//
//				// Save cal data to cartridge memory
//				MemoryWrite(Cal_page, OFFSET_T_CAL, 4, (uint8_t *) &T_Cal);
//				for(i = 0; i < 10; i++)
//				{
//					MemoryWrite(Cal_page, OFFSET_ISE_1_SLOPE + (i * 4), 4, (uint8_t *) &ISE_Slope_CalT[i]);
////					MemoryWrite(Cal_page, OFFSET_ISE_1_INT + (i * 4), 4, (uint8_t *) &ISE_Int[i]);
//					MemoryWrite(Cal_page, OFFSET_CR_ISE_1_RINSE + (i * 4), 4, (uint8_t *) &ISE_mV_Rinse[i]);
//					MemoryWrite(Cal_page, OFFSET_CR_ISE_1_POST + (i * 4), 4, (uint8_t *) &ISE_mV_PostRinse[i]);
//				}
//
//				MemoryWrite(Cal_page, OFFSET_COND_R1_SLOPE, 4, (uint8_t *) &CalConductivitySlopeLow);
//				MemoryWrite(Cal_page, OFFSET_COND_R1_INT, 4, (uint8_t *) &CalConductivityKLow);
//				MemoryWrite(Cal_page, OFFSET_COND_R2_SLOPE, 4, (uint8_t *) &CalConductivitySlopeMid);
//				MemoryWrite(Cal_page, OFFSET_COND_R2_INT, 4, (uint8_t *) &CalConductivityKMid);
//				MemoryWrite(Cal_page, OFFSET_COND_R3_SLOPE, 4, (uint8_t *) &CalConductivitySlopeHigh);
//				MemoryWrite(Cal_page, OFFSET_COND_R3_INT, 4, (uint8_t *) &CalConductivityKHigh);
//
//				MemoryWrite(Cal_page, OFFSET_CAL_STATUS, 4, (uint8_t *) &Calibration_Status);
//				MemoryWrite(Cal_page, OFFSET_CAL_DATA_ZERO, 1, &Zero);
//
//				// Parse together the calibrated byte for the app reporting
//				if((gui32Error & CL_CLEANING_OUT_OF_RANGE) == 0)
//					App_Status |= (1 << APP_CHLORINE);
//				MemoryWrite(Cal_page, OFFSET_CALIBRATED_STATUS, 1, &App_Status);
//
//				// Calculate and save the chosen slope percentages
//				if(ISEs.pH_H2.size > 0)
//				{
//					int16_t Slope_Percent = (float) ((pH_H2_Slope_CalT[L_pH_H2_Chosen] * (25.0 + 273.0)/(T_Cal + 273.0)) * 10000.0 / -59.9);	// Temperature correct slope to 25 then calculate it's percentage of theory
//					MemoryWrite(Cal_page, OFFSET_ALK_SLOPE_PER, 2, (uint8_t *) &Slope_Percent);
//				}
//				if(ISEs.pH_Cr.size > 0)
//				{
//					int16_t Slope_Percent = (float) ((pH_Cr_Slope_CalT[L_pH_Cr_Chosen] * (25.0 + 273.0)/(T_Cal + 273.0)) * 10000.0 / -59.9);	// Temperature correct slope to 25 then calculate it's percentage of theory
//					MemoryWrite(Cal_page, OFFSET_PH_SLOPE_PER, 2, (uint8_t *) &Slope_Percent);
//				}
//				if(ISEs.TH.size > 0)
//				{
//					int16_t Slope_Percent = (float) ((TH_Slope_CalT[L_TH_Chosen] * (25.0 + 273.0)/(T_Cal + 273.0)) * 10000.0 / -29.5);	// Temperature correct slope to 25 then calculate it's percentage of theory
//					MemoryWrite(Cal_page, OFFSET_MG_SLOPE_PER, 2, (uint8_t *) &Slope_Percent);
//				}
//				if(ISEs.NH4.size > 0)
//				{
//					int16_t Slope_Percent = (float) ((NH4_Slope_CalT[L_NH4_Chosen] * (25.0 + 273.0)/(T_Cal + 273.0)) * 10000.0 / -59.9);	// Temperature correct slope to 25 then calculate it's percentage of theory
//					MemoryWrite(Cal_page, OFFSET_NH4_SLOPE_PER, 2, (uint8_t *) &Slope_Percent);
//				}
//				if(ISEs.Ca.size > 0)
//				{
//					int16_t Slope_Percent = (float) ((Ca_Slope_CalT[L_Ca_Chosen] * (25.0 + 273.0)/(T_Cal + 273.0)) * 10000.0 / -29.5);	// Temperature correct slope to 25 then calculate it's percentage of theory
//					MemoryWrite(Cal_page, OFFSET_CA_SLOPE_PER, 2, (uint8_t *) &Slope_Percent);
//				}
//
//				// Calculate the conductivity slope percentage
//				if(1)
//				{
////					// Put the 3 calibrants used in the array twice each, order doesn't matter here because the array will be sorted from smallest to largest
////					// Really only need 5 points with the highest conductivity calibrant in the array once, but to make it universal have an extra spot and the last spot will be ignored after sorting
////					float CalConds[6] = {Sols->Cond_EEP_Clean*(1 + Sols->Clean_Cond_TComp*(T_Cal - 25))/1000000, Sols->Cond_EEP_Clean*(1 + Sols->Clean_Cond_TComp*(T_Cal - 25))/1000000, Sols->Cond_EEP_Cal_1*(1 + Sols->Cal_1_Cond_TComp*(T_Cal - 25))/1000000, Sols->Cond_EEP_Cal_1*(1 + Sols->Cal_1_Cond_TComp*(T_Cal - 25))/1000000, Sols->Cond_EEP_Cal_2*(1 + Sols->Cal_2_Cond_TComp*(T_Cal - 25))/1000000, Sols->Cond_EEP_Cal_2*(1 + Sols->Cal_2_Cond_TComp*(T_Cal - 25))/1000000};
////					SortArray(CalConds, 6);
////
////					float I_Low, I_Mid, I_High;
////
////					// Read the currents off the memory, these should be saved during the QC process
////					EEPROMRead((uint32_t *) &I_Low, OFFSET_COND_I_LOW, 4);
////					EEPROMRead((uint32_t *) &I_Mid, OFFSET_COND_I_MID, 4);
////					EEPROMRead((uint32_t *) &I_High, OFFSET_COND_I_HIGH, 4);
////
////					if(I_Low != I_Low)
////						I_Low = 10.76 * 0.795;	// Average from circuits before ARV1_0B
////					if(I_Mid != I_Mid)
////						I_Mid = 19.89 * 0.8;	// Average from circuits before ARV1_0B
////					if(I_High != I_High)
////						I_High = 43.57 * .812;	// Average from circuits before ARV1_0B
////
////					float MeasConds[5] = {I_Low/CalConductivityV2Low,I_Mid/CalConductivityV1Mid,I_Mid/CalConductivityV2Mid,I_High/CalConductivityV2High,I_High/CalConductivityV3High};
////
////					float CondFactorySlope = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_FACTORY_COND_SLOPE, 4));
////					if(CondFactorySlope == 0 || CondFactorySlope != CondFactorySlope)	// Check if there is valid data saved in the memory
////						CondFactorySlope = 0.18;
////
////					int16_t Slope_Percent = FindBestFitSlope(CalConds, MeasConds, 5)*1000000.0 * 10000.0 / CondFactorySlope;	// Temperature correct slope to 25 then calculate it's percentage of theory
//
//
//
//					// Put the 3 calibrants used in the array twice each, order doesn't matter here because the array will be sorted from smallest to largest
//					// Really only need 5 points with the highest conductivity calibrant in the array once, but to make it universal have an extra spot and the last spot will be ignored after sorting
//					float CalConds[3] = {Sols->Cond_EEP_Clean*(1 + Sols->Clean_Cond_TComp*(T_Cal - 25)), Sols->Cond_EEP_Cal_1*(1 + Sols->Cal_1_Cond_TComp*(T_Cal - 25)), Sols->Cond_EEP_Cal_2*(1 + Sols->Cal_2_Cond_TComp*(T_Cal - 25))};
//					SortArray(CalConds, 3);
//
//					float I_Low, I_Mid, I_High;
//
//					// Read the currents off the memory, these should be saved during the QC process
//					EEPROMRead((uint32_t *) &I_Low, OFFSET_COND_I_LOW, 4);
//					EEPROMRead((uint32_t *) &I_Mid, OFFSET_COND_I_MID, 4);
//					EEPROMRead((uint32_t *) &I_High, OFFSET_COND_I_HIGH, 4);
//
//					if(I_Low != I_Low)
//						I_Low = 10.76 * 0.795;	// Average from circuits before ARV1_0B
//					if(I_Mid != I_Mid)
//						I_Mid = 19.89 * 0.8;	// Average from circuits before ARV1_0B
//					if(I_High != I_High)
//						I_High = 43.57 * .812;	// Average from circuits before ARV1_0B
//
//					float CondFactorySlope = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_FACTORY_COND_SLOPE, 4));
//					if(CondFactorySlope == 0 || CondFactorySlope != CondFactorySlope)	// Check if there is valid data saved in the memory
//						CondFactorySlope = 0.18;
//
//					int16_t Slope_Percent = ((I_High * 1000000.0 / CalConductivityV3High - I_Mid * 1000000.0 / CalConductivityV2Mid) / (CalConds[2] - CalConds[1])) * 10000.0 / CondFactorySlope;	// Temperature correct slope to 25 then calculate it's percentage of theory
//
//					MemoryWrite(Cal_page, OFFSET_COND_SLOPE_PER, 2, (uint8_t *) &Slope_Percent);
//					DEBUG_PRINT(UARTprintf("Cond Slope Percentage: %d / 100\n", Slope_Percent);)
//				}
//
//				MemoryWrite(Cal_page, OFFSET_CAL_CHOSEN_SENSORS, 1, &L_Chosen_Sensors);
//
//				MemoryWrite(Cal_page, OFFSET_CAL_LOG_K, 4, (uint8_t *) &log_K_Ca_Mg);
//
//				MemoryWrite(Cal_page, OFFSET_CR_ZERO, 1, &Zero);
//
//				MemoryWrite(Cal_page, OFFSET_CR_T_POSTRINSE, 4, (uint8_t *) &T_PostRinse);
//
//				// Save error data after all other data has been written to memory
//				gui32Error &= ~(ROAM_RESET | APP_FILTER_ERROR);	// Turn off ROAM_RESET flag before saving error code
//				MemoryWrite(Cal_page, OFFSET_CAL_ERROR, 4, (uint8_t *) &gui32Error);
//
//#ifdef PRINT_UART
//				if(PRINT_RAW == 1)
//				{
//					// Print out raw calibration data to terminal
//					DEBUG_PRINT(UARTprintf("Raw Calibration Data:\n");)
//					DEBUG_PRINT(UARTprintf("Sensor\tRinse\tCal 1\tCal 2\tClean\tPostrinse\n");)
//					for(i = 0; i < ISEs.pH_H2.size; i++)
//						{DEBUG_PRINT(UARTprintf("pH H2 %d\t%d\t%d\t%d\t%d\t%d\n", i + 1, (int) (pH_H2_mV_Rinse[i] * 1000), (int) (pH_H2_mV_Cal_1[i] * 1000), (int) (pH_H2_mV_Cal_2[i] * 1000), (int) (pH_H2_mV_Clean[i] * 1000), (int) (pH_H2_mV_PostRinse[i] * 1000));)}
//					for(i = 0; i < ISEs.pH_Cr.size; i++)
//						{DEBUG_PRINT(UARTprintf("pH %d\t%d\t%d\t%d\t%d\t%d\n", i + 1, (int) (pH_Cr_mV_Rinse[i] * 1000), (int) (pH_Cr_mV_Cal_1[i] * 1000), (int) (pH_Cr_mV_Cal_2[i] * 1000), (int) (ISE_mV_Clean[i + ISEs.pH_Cr.index] * 1000), (int) (pH_Cr_mV_PostRinse[i] * 1000));)}
//					for(i = 0; i < ISEs.TH.size; i++)
//						{DEBUG_PRINT(UARTprintf("TH %d\t%d\t%d\t%d\t%d\t%d\n", i + 1, (int) (TH_mV_Rinse[i] * 1000), (int) (TH_mV_Cal_1[i] * 1000), (int) (TH_mV_Cal_2[i] * 1000), (int) (ISE_mV_Clean[i + ISEs.TH.index] * 1000), (int) (TH_mV_PostRinse[i] * 1000));)}
//					for(i = 0; i < ISEs.NH4.size; i++)
//						{DEBUG_PRINT(UARTprintf("NH4 %d\t%d\t%d\t%d\t%d\t%d\n", i + 1, (int) (NH4_mV_Rinse[i] * 1000), (int) (NH4_mV_Cal_1[i] * 1000), (int) (NH4_mV_Cal_2[i] * 1000), (int) (NH4_mV_Clean[i] * 1000), (int) (NH4_mV_PostRinse[i] * 1000));)}
//					for(i = 0; i < ISEs.Ca.size; i++)
//						{DEBUG_PRINT(UARTprintf("Ca %d\t%d\t%d\t%d\t%d\t%d\n", i + 1, (int) (Ca_mV_Rinse[i] * 1000), (int) (Ca_mV_Cal_1[i] * 1000), (int) (Ca_mV_Cal_2[i] * 1000), (int) (ISE_mV_Clean[i + ISEs.Ca.index] * 1000), (int) (Ca_mV_PostRinse[i] * 1000));)}
//					DEBUG_PRINT(UARTprintf("\n");)
//
//					DEBUG_PRINT(UARTprintf("Raw Conductivity Data: \n");)
//					DEBUG_PRINT(UARTprintf("Cond Low\t%d\n", (int) (CalConductivityV2Low * 1000));)
//					DEBUG_PRINT(UARTprintf("Cond Mid\t%d\t%d\n", (int) (CalConductivityV1Mid * 1000), (int) (CalConductivityV2Mid * 1000));)
//					DEBUG_PRINT(UARTprintf("Cond High\t%d\t%d\n\n", (int) (CalConductivityV2High * 1000), (int) (CalConductivityV3High * 1000));)
//
//					DEBUG_PRINT(UARTprintf("Raw Conductivity Data: \n");)
//					DEBUG_PRINT(UARTprintf("Cond Low\t%d\t%d\n", (int) (CalConductivityV1LowInv * 1000), (int) (CalConductivityV2LowInv * 1000));)
//					DEBUG_PRINT(UARTprintf("Cond Mid\t%d\t%d\n", (int) (CalConductivityV1MidInv * 1000), (int) (CalConductivityV2MidInv * 1000));)
//					DEBUG_PRINT(UARTprintf("Cond High\t%d\t%d\n\n", (int) (CalConductivityV2HighInv * 1000), (int) (CalConductivityV3HighInv * 1000));)
//
//					DEBUG_PRINT(UARTprintf("Calculated p-values:\n");)
//					DEBUG_PRINT(UARTprintf("\tRinse\tCal 1\tCal 2\tClean\n");)
//					if(ISEs.pH_H2.size > 0 || ISEs.pH_Cr.size > 0)
//						{DEBUG_PRINT(UARTprintf("pH\t%d\t%d\t%d\t%d\n", (int) (pH_TCor_Rinse * 1000), (int) (pH_TCor_Cal_1 * 1000), (int) (pH_TCor_Cal_2 * 1000), (int) (pH_TCor_Clean * 1000));)}
//					if(ISEs.TH.size > 0)
//						{DEBUG_PRINT(UARTprintf("pTH\t%d\t%d\t%d\t%d\n", (int) (pTH_Rinse * 1000), (int) (pTH_Cal_1 * 1000), (int) (pTH_Cal_2 * 1000), (int) (pTH_Clean * 1000));)}
//					if(ISEs.NH4.size > 0)
//						{DEBUG_PRINT(UARTprintf("pNH4\t%d\t%d\t%d\t%d\n", (int) (pNH4_Rinse * 1000), (int) (pNH4_Cal_1 * 1000), (int) (pNH4_Cal_2 * 1000), (int) (pNH4_Clean * 1000));)}
//					if(ISEs.Ca.size > 0)
//						{DEBUG_PRINT(UARTprintf("pCa\t%d\t%d\t%d\t%d\n", (int) (pCa_Rinse * 1000), (int) (pCa_Cal_1 * 1000), (int) (pCa_Cal_2 * 1000), (int) (pCa_Clean * 1000));)}
//#ifdef TH_ITERATED_MATH
//					if(ISEs.TH.size > 0)
//						{DEBUG_PRINT(UARTprintf("pMg\t%d\t\t%d\n", (int) (pMg_Rinse * 1000), (int) (pMg_Cal_2 * 1000));)}
//#endif
//
//					DEBUG_PRINT(UARTprintf("Temperatures:\n");)
//					DEBUG_PRINT(UARTprintf("Prerinse\t%d\n", (int) (T_Rinse * 1000));)
//					DEBUG_PRINT(UARTprintf("Cal 1\t%d\n", (int) (T_Cal_1 * 1000));)
//					DEBUG_PRINT(UARTprintf("Cal 2\t%d\n", (int) (T_Cal_2 * 1000));)
//					if(ISEs.Config != PH_CL_CART)
//						{DEBUG_PRINT(UARTprintf("Clean\t%d\n", (int) (T_Clean * 1000));)}
//				}
//
//				if(Sols->pH_EEP_Cal_2 < 9)
//				{
//					DEBUG_PRINT(UARTprintf("Chosen Sensors by slope:\n");)
//					DEBUG_PRINT(UARTprintf("pH H2: %d\n", L_pH_H2_Chosen);)
//					DEBUG_PRINT(UARTprintf("pH Cr: %d\n", L_pH_Cr_Chosen);)
//					DEBUG_PRINT(UARTprintf("TH: %d\n", L_TH_Chosen);)
//					DEBUG_PRINT(UARTprintf("NH4: %d\n", L_NH4_Chosen);)
//					DEBUG_PRINT(UARTprintf("Ca: %d\n\n", L_Ca_Chosen);)
//				}
//				else
//				{
//					DEBUG_PRINT(UARTprintf("Linear Chosen Sensors:\n");)
//					if(ISEs.pH_H2.size > 0)
//						{DEBUG_PRINT(UARTprintf("pH H2 sensor doesn't have a third point to check linearity against, can't pick sensor!\n");)}
//					DEBUG_PRINT(UARTprintf("pH Cr: %d\n", L_pH_Cr_Chosen);)
//					DEBUG_PRINT(UARTprintf("TH: %d\n", L_TH_Chosen);)
//					DEBUG_PRINT(UARTprintf("NH4: %d\n", L_NH4_Chosen);)
//					DEBUG_PRINT(UARTprintf("Ca: %d\n\n", L_Ca_Chosen);)
//				}
//
//
//#ifdef PH_LOG_K
//				// Print out calculated slopes to terminal
//				DEBUG_PRINT(UARTprintf("Slopes:\n");)
//				for(i = 0; i < ISEs.pH_H2.size; i++)
//					{DEBUG_PRINT(UARTprintf("pH H2 %d\t%d\n", (i + 1), (int) (pH_H2_Slope_CalT[i] * 1000));)}
//				for(i = 0; i < ISEs.pH_Cr.size; i++)
//					{DEBUG_PRINT(UARTprintf("pH Cr %d\t%d\n", (i + 1), (int) (pH_Cr_Slope_CalT[i] * 1000));)}
//				for(i = 0; i < ISEs.TH.size; i++)
//					{DEBUG_PRINT(UARTprintf("TH %d\t%d\tLog K\t%d\n", (i + 1), (int) (TH_Slope_CalT[i] * 1000), (int) (Build_float(MemoryRead(Cal_page, OFFSET_TH_1_LOG_K + (i * 4), 4)) * 1000));)}
//
//				for(i = 0; i < ISEs.NH4.size; i++)
//					{DEBUG_PRINT(UARTprintf("NH4 %d\t%d\n", (i + 1), (int) (NH4_Slope_CalT[i] * 1000));)}
//				for(i = 0; i < ISEs.Ca.size; i++)
//					{DEBUG_PRINT(UARTprintf("Ca %d\t%d\tLog K\t%d\n", (i + 1), (int) (Ca_Slope_CalT[i] * 1000), (int) (Build_float(MemoryRead(Cal_page, OFFSET_CA_1_LOG_K + (i * 4), 4)) * 1000));)}
//#ifdef TH_ITERATED_MATH
//				if(ISEs.TH.size > 0)
//				{
//					for(i = 0; i < 2; i++)
//					{
//						DEBUG_PRINT(UARTprintf("Mg %d\t%d\n", (i + 1), (int) (Mg_Slope_CalT[i] * 1000));)
//					}
//				}
//#endif	// TH_ITERATED_MATH
//#else	// PH_LOG_K
//				// Print out calculated slopes to terminal
//				DEBUG_PRINT(UARTprintf("Slopes:\n");)
//				for(i = 0; i < ISEs.pH_H2.size; i++)
//					{DEBUG_PRINT(UARTprintf("pH H2 %d\t%d\n", (i + 1), (int) (pH_H2_Slope_CalT[i] * 1000));)}
//				for(i = 0; i < ISEs.pH_Cr.size; i++)
//					{DEBUG_PRINT(UARTprintf("pH Cr %d\t%d\n", (i + 1), (int) (pH_Cr_Slope_CalT[i] * 1000));)}
//#ifdef LINEAR_PH_CORR
//				for(i = 0; i < ISEs.TH.size; i++)
//					{DEBUG_PRINT(UARTprintf("TH %d\t%d\t%d\n", (i + 1), (int) (TH_Slope_CalT[i] * 1000), (int) (Build_float(MemoryRead(Cal_page, OFFSET_MG_1_PH_SLOPE + (i * 4), 4)) * 1000));)}
//#else	// LINEAR_PH_CORR
//				for(i = 0; i < ISEs.TH.size; i++)
//					{DEBUG_PRINT(UARTprintf("TH %d\t%d\t%d\n", (i + 1), (int) (TH_Slope_CalT[i] * 1000));)}
//#endif	// LINEAR_PH_CORR
//
//#ifdef NH4_PH_CORR
//				for(i = 0; i < ISEs.NH4.size; i++)
//					{DEBUG_PRINT(UARTprintf("NH4 %d\t%d\t%d\n", (i + 1), (int) (NH4_Slope_CalT[i] * 1000), (int) (Build_float(MemoryRead(Cal_page, OFFSET_NH4_1_LOG_K + (i * 4), 4)) * 1000));)}
//#else
//				for(i = 0; i < ISEs.NH4.size; i++)
//					{DEBUG_PRINT(UARTprintf("NH4 %d\t%d\n", (i + 1), (int) (NH4_Slope_CalT[i] * 1000));)}
//#endif
//
//#ifdef LINEAR_PH_CORR
//				for(i = 0; i < ISEs.Ca.size; i++)
//					{DEBUG_PRINT(UARTprintf("Ca %d\t%d\t%d\n", (i + 1), (int) (Ca_Slope_CalT[i] * 1000), (int) (Build_float(MemoryRead(Cal_page, OFFSET_CA_1_LOG_K + (i * 4), 4)) * 1000));)}
//#else // LINEAR_PH_CORR
//				for(i = 0; i < ISEs.Ca.size; i++)
//					{DEBUG_PRINT(UARTprintf("Ca %d\t%d\n", (i + 1), (int) (Ca_Slope_CalT[i] * 1000));)}
//#endif// LINEAR_PH_CORR
//#ifdef TH_ITERATED_MATH
//				if(ISEs.TH.size > 0)
//				{
//					for(i = 0; i < 2; i++)
//					{
//						DEBUG_PRINT(UARTprintf("Mg %d\t%d\n", (i + 1), (int) (Mg_Slope_CalT[i] * 1000));)
//					}
//				}
//#endif	// TH_ITERATED_MATH
//#endif	// PH_LOG_K
//
//				DEBUG_PRINT(UARTprintf("Cond Low\t%d\t%d\n", (int) (CalConductivitySlopeLow * 1000), (int) (CalConductivityKLow * 1000));)
//				DEBUG_PRINT(UARTprintf("Cond Mid\t%d\t%d\n", (int) (CalConductivitySlopeMid * 1000), (int) (CalConductivityKMid * 1000));)
//				DEBUG_PRINT(UARTprintf("Cond High\t%d\t%d\n", (int) (CalConductivitySlopeHigh * 1000), (int) (CalConductivityKHigh * 1000));)
//
//				DEBUG_PRINT(UARTprintf("\n");)
//
//				// Print out chosen slopes to terminal
//				DEBUG_PRINT(UARTprintf("Chosen slopes:\n");)
//				if(ISEs.pH_H2.size > 0)
//					{DEBUG_PRINT(UARTprintf("pH H2\t%d\n", (int) (pH_H2_Slope_CalT[L_pH_H2_Chosen] * 1000));)}
//				if(ISEs.pH_Cr.size > 0)
//					{DEBUG_PRINT(UARTprintf("pH\t%d\n", (int) (pH_Cr_Slope_CalT[L_pH_Cr_Chosen] * 1000));)}
//#ifdef LINEAR_PH_CORR
//				if(ISEs.TH.size > 0)
//					{DEBUG_PRINT(UARTprintf("TH\t%d\t%d\n", (int) (TH_Slope_CalT[L_TH_Chosen] * 1000), (int) (Build_float(MemoryRead(Cal_page, OFFSET_MG_1_PH_SLOPE + (L_TH_Chosen * 4), 4)) * 1000));)}
//#else
//				if(ISEs.TH.size > 0)
//					{DEBUG_PRINT(UARTprintf("TH\t%d\n", (int) (TH_Slope_CalT[L_TH_Chosen] * 1000));)}
//#endif
//
//#ifdef NH4_PH_CORR
//				if(ISEs.NH4.size > 0)
//					{DEBUG_PRINT(UARTprintf("NH4\t%d\t%d\n", (int) (NH4_Slope_CalT[L_NH4_Chosen] * 1000), (int) (Build_float(MemoryRead(Cal_page, OFFSET_NH4_1_LOG_K + (L_NH4_Chosen * 4), 4)) * 1000));)}
//#else
//				if(ISEs.NH4.size > 0)
//					{DEBUG_PRINT(UARTprintf("NH4\t%d\n", (int) (NH4_Slope_CalT[L_NH4_Chosen] * 1000));)}
//#endif
//
//#ifdef LINEAR_PH_CORR
//				if(ISEs.Ca.size > 0)
//					{DEBUG_PRINT(UARTprintf("Ca\t%d\t%d\n", (int) (Ca_Slope_CalT[L_Ca_Chosen] * 1000), (int) (Build_float(MemoryRead(Cal_page, OFFSET_CA_1_LOG_K + (L_Ca_Chosen * 4), 4)) * 1000));)}
//#else
//				if(ISEs.Ca.size > 0)
//					{DEBUG_PRINT(UARTprintf("Ca\t%d\n", (int) (Ca_Slope_CalT[L_Ca_Chosen] * 1000));)}
//#endif
//
//				DEBUG_PRINT(UARTprintf("\n");)
//
//				DEBUG_PRINT(UARTprintf("Sensor calibration status: \n");)
//
//				if(ISEs.pH_H2.size > 0)
//				{
//					DEBUG_PRINT(UARTprintf("pH H2:");)
//					for(i = 0; i < ISEs.pH_H2.size; i++)
//						{DEBUG_PRINT(UARTprintf("\t%d", pH_H2_Cal_Check[i]);)}
//					DEBUG_PRINT(UARTprintf("\n");)
//				}
//				if(ISEs.pH_Cr.size > 0)
//				{
//					DEBUG_PRINT(UARTprintf("pH Cr:");)
//					for(i = 0; i < ISEs.pH_Cr.size; i++)
//						{DEBUG_PRINT(UARTprintf("\t%d", pH_Cr_Cal_Check[i]);)}
//					DEBUG_PRINT(UARTprintf("\n");)
//				}
//				if(ISEs.TH.size > 0)
//				{
//					DEBUG_PRINT(UARTprintf("TH:");)
//					for(i = 0; i < ISEs.TH.size; i++)
//						{DEBUG_PRINT(UARTprintf("\t%d", TH_Cal_Check[i]);)}
//					DEBUG_PRINT(UARTprintf("\n");)
//				}
//				if(ISEs.NH4.size > 0)
//				{
//					DEBUG_PRINT(UARTprintf("NH4:");)
//					for(i = 0; i < ISEs.NH4.size; i++)
//						{DEBUG_PRINT(UARTprintf("\t%d", NH4_Cal_Check[i]);)}
//					DEBUG_PRINT(UARTprintf("\n");)
//				}
//				if(ISEs.Ca.size > 0)
//				{
//					DEBUG_PRINT(UARTprintf("Ca:");)
//					for(i = 0; i < ISEs.Ca.size; i++)
//						{DEBUG_PRINT(UARTprintf("\t%d", Ca_Cal_Check[i]);)}
//					DEBUG_PRINT(UARTprintf("\n");)
//				}
//
//				if(Cal_failed == 0)
//					{DEBUG_PRINT(UARTprintf("Calibration Passed!\n");)}
//				else
//					{DEBUG_PRINT(UARTprintf("Calibration Failed!\n");)}
//
//#endif
//
//				uint16_t No_of_cals = *((uint16_t *) MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_COMPLETED_CALS, 2));
//				if(No_of_cals == 0xFFFF)
//					No_of_cals = 0;
//
//				No_of_cals++;
//				MemoryWrite(PAGE_CARTRIDGE_INFO, OFFSET_COMPLETED_CALS, 2, (uint8_t *) &No_of_cals);
//
//				DEBUG_PRINT(UARTprintf("%d calibrations performed on this sensor! \n\n", No_of_cals);)
//			}
//			else
//				{DEBUG_PRINT(UARTprintf("Calibration Aborted!\n");)}

			TestValveDrift();
#ifdef LOOSE_VALVE
			TurnValveToStore(V_STORE);
#endif

			if((gui32Error & ABORT_ERRORS) == 0)
			{
				uint16_t No_of_cals = *((uint16_t *) MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_COMPLETED_CALS, 2));
				if(No_of_cals == 0xFFFF)
					No_of_cals = 0;

				No_of_cals++;
				MemoryWrite(PAGE_CARTRIDGE_INFO, OFFSET_COMPLETED_CALS, 2, (uint8_t *) &No_of_cals);

				DEBUG_PRINT(UARTprintf("%d calibrations performed on this sensor! \n\n", No_of_cals);)
			}

			// Save error data after all other data has been written to memory
			gui32Error &= ~(ROAM_RESET | APP_FILTER_ERROR);	// Turn off ROAM_RESET flag before saving error code
			MemoryWrite(Cal_page, OFFSET_CAL_ERROR, 4, (uint8_t *) &gui32Error);

			DEBUG_PRINT(UARTprintf("Error Code: 0x%x\n", gui32Error);)
			PrintErrors(gui32Error, 1, STATE_CALIBRATION);

			update_Cal(Cal_Number);

			//			FindPossitionOneValve();

			// Wait at end of calibration set current operation to successful cal
			// Exit this by either timeout (return to idle), button (idle or test), or BT command (idle or test)
			update_Status(STATUS_CALIBRATION, OPERATION_EMPTY_WASTE);	// Waiting to empty waste chamber
			DEBUG_PRINT(UARTprintf("Empty waste chamber! Press button to continue. \n");)
			g_ulSSI0RXTO = 0;

			SetLED(BLUE_BUTTON | BLUE_BUTTON_BLINK | BLUE_BUTTON_V | BLUE_BUTTON_BLINK_V, 0);
			if(Calibration_Status & 1)	// Calibration Passed, blink green LED
				SetLED(GREEN_BUTTON | GREEN_BUTTON_V, 1);
			else
				SetLED(RED_BUTTON | RED_BUTTON_V, 1);

			counter = 0;
			while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == IO_BUTTON_PIN && counter < TIMEOUT)
//			while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == IO_BUTTON_PIN && counter < 600000)
			{
				SysCtlDelay(SysCtlClockGet()/3000);
				counter++;

				// Break out of while loop if continue calibration command is received
				if(g_ui32DataRx0[0] == CONTINUE_CAL && g_ulSSI0RXTO > 0)
				{
					g_ulSSI0RXTO = 0;
					break;
				}

				// Check if state changed, this happens when abort command is received
				if(g_state != STATE_CALIBRATION)
					break;	// Breaks out of while loop
			}

			if(Calibration_Status & 1)
				update_Status(STATUS_CALIBRATION, OPERATION_CAL_COMPLETE);
			else
				update_Status(STATUS_CALIBRATION, OPERATION_CAL_FAILED);
			SetLED(RED_BUTTON | GREEN_BUTTON | RED_BUTTON_V | GREEN_BUTTON_V | BLUE_BUTTON_V | LED_BLINK, 0);
			while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == 0x00);

			counter = 0;
			g_ulSSI0RXTO = 0;
			while(counter < 5000)	// Wait 5 seconds, or until the app acknowledges it is done
			{
				SysCtlDelay(SysCtlClockGet()/3000);
				counter++;

				// Break out of while loop if continue calibration command is received
				if(g_ui32DataRx0[0] == CONTINUE_CAL && g_ulSSI0RXTO > 0)
				{
					g_ulSSI0RXTO = 0;
					break;
				}

				// Check if state changed, this happens when abort command is received
				if(g_state != STATE_CALIBRATION)
					break;	// Breaks out of while loop
			}

//			// Wait 5 ms to let app read status and operation before switching back to idle
//			SysCtlDelay(SysCtlClockGet()/3000 * 5);

			g_state = STATE_IDLE;
			g_next_state = STATE_IDLE;

			break;
		}
		case STATE_MEASUREMENT:
		{
			// Must send status before updating error so we know where to assign error
//			update_Status(STATUS_TEST, OPERATION_TEST_PRECHECK);	// Send status to BT that this is pre-check

			// Set error to 0 at beginning
			gui32Error = 0;
			update_Error();

			// Precheck to make sure everything is good to run calibration
			gui32Error = ROAM_RESET | APP_FILTER_ERROR;	// Set global error to ROAM_RESET so its only error at beginning so all errors that appear are during running test
			pui8SysStatus = RequestSystemStatus(); // Get time and user information at beginning of test
			CheckCartridge(pui8SysStatus);
			uint8_t Battery_Percent = BatteryRead(REP_SOC_REG);
			if(Battery_Percent < MIN_BAT_LEVEL && (GPIOPinRead(IO_POWER_GOOD_B_BASE, IO_POWER_GOOD_B_PIN) != 0 && gBoard >= V6_2))	// Check that battery is at minimum level or device is plugged in
			{
				DEBUG_PRINT(UARTprintf("Battery is too low! Aborting Test!\n");)
				gui32Error |= BATTERY_TOO_LOW;
			}

			uint32_t counter = 0;

			// Check for error that would prevent us from starting calibration, blink light red if this occurs
			//			if(gui32Error != ROAM_RESET)	// Something in the cartridge failed (max number of tests exceeded or cartridge expired) or battery check failed
			if((gui32Error & STARTING_ERRORS) != 0 || gCartridge == 0)
			{
				if(ENFORCE_ERRORS != 0)
				{
					update_Status(STATUS_TEST, OPERATION_TEST_FAILED);	// Send status to BT that this is pre-check

					gui32Error &= ~(ROAM_RESET | APP_FILTER_ERROR);	// Remove ROAM_RESET flag since software stopped calibration
					update_Error();
					PrintErrors(gui32Error, 1, STATE_MEASUREMENT);
					if(gCartridge == 0)
						{DEBUG_PRINT(UARTprintf("No cartridge plugged in!\n");)}

					SetLED(GREEN_BUTTON | GREEN_BUTTON_V, 0);
					SetLED(RED_BUTTON | RED_BUTTON_V, 1);
					while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == IO_BUTTON_PIN && counter < TIMEOUT)
					{
						SysCtlDelay(SysCtlClockGet()/3000);
						counter++;

						// Break out of while loop if continue calibration command is received
						if(g_ui32DataRx0[0] == CONTINUE_TEST && g_ulSSI0RXTO > 0)
						{
							g_ulSSI0RXTO = 0;
							break;
						}

						// Return to idle if abort command is received
						if(g_state != STATE_MEASUREMENT)
						{
							break;
						}
					}

					while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == 0);

					g_state = STATE_IDLE;
					g_next_state = STATE_IDLE;
					break;
				}
			}

			// Read sensor configuration from memory so program knows what type of test to run
			//			uint8_t Sensor_Config = *MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_CONFIGURATION, 1);
			struct ISEConfig ISEs;
			FillISEStruct(&ISEs);

			//			uint8_t Sensor_Config = *MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_SENSOR_CONFIGURATION, 1);
			//			uint32_t Days_between_calibration = 1;
			//			if(Sensor_Config == PH_CL_CART)
			//				Days_between_calibration = 2;

			if(CheckCalibration(pui8SysStatus, ISEs.CalDelay) == 1 || DEMO_UNIT == 1)
				SetLED(GREEN_BUTTON | GREEN_BUTTON_V, 1);
			else
			{
				SetLED(BLUE_BUTTON | BLUE_BUTTON_V, 1);
				update_Status(STATUS_CALIBRATION, OPERATION_CAL_PRECHECK);
#ifndef TESTING_MODE
				g_state = STATE_CALIBRATION;
//				g_next_state = STATE_MEASUREMENT;

				counter = 0;
				while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == IO_BUTTON_PIN && counter < TIMEOUT)
				{
					SysCtlDelay(SysCtlClockGet()/3000);
					counter++;

					// Break out of loop if continue test command is received
					if(g_ui32DataRx0[0] == CONTINUE_TEST && g_ulSSI0RXTO > 0)
					{
						g_state = STATE_MEASUREMENT;	// Continue test command is only way to continue test is calibration has failed and NOT in testing mode
						g_ulSSI0RXTO = 0;
						SetLED(BLUE_BUTTON | BLUE_BUTTON_V, 0);
						SetLED(GREEN_BUTTON | GREEN_BUTTON_V, 1);
						break;
					}

					// Break out of loop if continue test command is received
					if(g_ui32DataRx0[0] == CONTINUE_CAL && g_ulSSI0RXTO > 0)
					{
						g_ulSSI0RXTO = 0;
						break;
					}

					if(gCartridge == 0)	// If the cartridge is unplugged return to idle
						g_state = STATE_IDLE;

					// Return to idle if abort command is received
					if(g_state == STATE_IDLE)
						break;
				}

				if(counter == TIMEOUT)
				{
					g_state = STATE_IDLE;
					g_next_state = STATE_IDLE;
				}

				SetLED(BLUE_BUTTON | BLUE_BUTTON_V, 0);
#endif
			}



			// Return to idle if abort command is received
			if(g_state != STATE_MEASUREMENT)
				break;

			update_Status(STATUS_TEST, OPERATION_TEST_PRECHECK);	// Send status to BT that this is pre-check

			counter = 0;
#ifndef LIFETIME_TESTING
			while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == IO_BUTTON_PIN && counter < (TIMEOUT * 5))
			{
				SysCtlDelay(SysCtlClockGet()/3000);
				counter++;

				// Break out of loop if continue test command is received
				if(g_ui32DataRx0[0] == CONTINUE_TEST && g_ulSSI0RXTO > 0)
				{
					g_state = STATE_MEASUREMENT;	// Continue test command is only way to continue test is calibration has failed and NOT in testing mode
					g_ulSSI0RXTO = 0;
					break;
				}

				// Break out of loop if continue test command is received
				if(g_state == STATE_CALIBRATION && g_ui32DataRx0[0] == CONTINUE_CAL && g_ulSSI0RXTO > 0)
				{
					g_ulSSI0RXTO = 0;
					break;
				}

				if(gCartridge == 0)	// If the cartridge is unplugged return to idle
					g_state = STATE_IDLE;

//				// Return to idle if abort command is received
//				if(g_state != STATE_MEASUREMENT)
//					break;

				// Return to idle if abort command is received
				if(g_state == STATE_IDLE)
					break;
			}
#endif // LIFETIME_TESTING

			SetLED(GREEN_BUTTON | GREEN_BUTTON_V | RED_BUTTON | RED_BUTTON_V | BLUE_BUTTON | BLUE_BUTTON_V, 0);
			if(g_state == STATE_MEASUREMENT)
				SetLED(GREEN_BUTTON_BLINK, 1);

			while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == 0);

			if(counter == (TIMEOUT * 5))
				g_state = STATE_IDLE;

			// Return to idle if abort command is received
			if(g_state != STATE_MEASUREMENT)
				break;

#ifdef TESTING_MODE
			TimerIntDisable(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);
			IntDisable(INT_WTIMER0A);
			TimerLoadSet64(WTIMER0_BASE, 0xFFFFFFFFFFFFFFFF); // Set timer for 5 minutes, if timer expires hibernate device
			TimerConfigure(WTIMER0_BASE, TIMER_CFG_PERIODIC_UP);
			TimerEnable(WTIMER0_BASE, TIMER_BOTH);

			uint64_t start_clock = TimerValueGet64(WTIMER0_BASE);
#endif

			update_Status(STATUS_TEST, OPERATION_TEST_RINSE);

			//			// TODO: Set pump variables for test
			// Variables to control pump and valve
			uint8_t Number_of_bubbles_Prerinse = 3;// - Rinse_pumped;
			uint8_t Number_of_bubbles_samp = 4;
			uint8_t Number_of_bubbles_Postrinse = 2;
			//			if(Sensor_Config == PH_CL_CART)
			//				Number_of_bubbles_Postrinse = 4;
//			uint16_t runSteps_air_bubble = 2000;// * gPump_Ratio;	// Number of pump steps per air bubble
//			uint16_t Steps_Large_air_bubble = 5000;// * gPump_Ratio;		// Large air bubble on pre/post rinse
//			uint16_t Steps_Large_air_bubble_samp = 5000;// * gPump_Ratio;	// Large air bubble on sample
//			uint16_t runSteps_Solution_Prerinse = 2000;// * gPump_Ratio;	// Number of pump steps for solution between air bubbles
//			uint16_t runSteps_Solution_samp = 2000;// * gPump_Ratio;	// Number of pump steps for solution between air bubbles
//			uint16_t runSteps_Solution_Postrinse = 2000;// * gPump_Ratio;	// Number of pump steps for solution between air bubbles
//			uint16_t runSteps_Clean = 7000;
//			uint16_t runSteps_Clean_center = 7300;
//
//			uint16_t runSteps_PreRinse = (6000);// * gPump_Ratio;	// Number of pump steps to push in prerinse after air bubbles
//			uint16_t runSteps_Sample = (8000);// * gPump_Ratio;		// Number of pump steps to push in sample after bubbles
//			uint16_t runSteps_PostRinse = (6000);// * gPump_Ratio;	// Number of pump steps to push in postrinse after air bubbles
//			uint16_t runSteps_plug = 8250;// * gPump_Ratio;		// Number of steps to center the sample on the sensors and reference.
//			uint16_t runSteps_plug_samp = 7500;
//			uint32_t Steps_sample_rinse = 50000;// * gPump_Ratio;		// How much sample to pull before metering, large to clear air out of chip before metering buffers
//
//			//			uint32_t Steps_Sample_Prime = 275000;// * gPump_Ratio;	// Number of pump steps to prime sample tube before initial rinse
//			uint16_t Steps_Sample_Prime = 24000;// * gPump_Ratio;	// Number of pump steps to prime sample tube before initial rinse
//			//			uint16_t Steps_Sample_Purge = 20000;	// Number of pump steps to purge sample tube after post rinse but before store
//
//			// Variables to control mixing
//			uint16_t diffusion_time = 5000;				// Time to wait after oscillating in mixing chamber
//			uint8_t mix_cycles = 10; //10;				// Number of forward backward pump cycles
//			uint16_t Steps_cycles = 1500;// * gPump_Ratio;			// Steps to pump forward/backward
//
////			// Variables to control store at end
////			uint16_t Steps_Store_1 = 4250;// * gPump_Ratio;
////			uint16_t Steps_Store_air_1 = 2000;// * gPump_Ratio;
////			uint16_t Steps_Store_2 = 4000;// * gPump_Ratio;
////			uint16_t Steps_Store_air_2 = 4750;// * gPump_Ratio;
//
//			// Priming B1
//			uint16_t Steps_tube_bubble = 1000;
//			uint16_t Steps_tube_prime_buffers = 2000;
//			uint16_t Steps_tube_prime = 1000;
//
//			uint16_t Test_Number = FindTestNumber() + 1; // Add 1 to go to next free spot in memory
//
//			// Read step values from memory
//			uint16_t Steps_B1 = 480;	// Adjust this number for B1
//			uint16_t Steps_B2 = 480;	// Adjust this number for buffer
//
//			// Variable to control pumping T1
//			uint16_t Steps_PreT1 = 3000;// * gPump_Ratio;			// Number of pump steps to pull sample before buffer
//			uint16_t Steps_PostT1 = 4000;// * gPump_Ratio;		// Number of pump steps to pull sample after buffer
//			uint16_t Steps_follow_T1 = 1100;// * gPump_Ratio;	// Steps to pump sample to get B1 mixture into mixing chamber
//			uint16_t Steps_center_T1 = 7900;// * gPump_Ratio;		// Steps to pump to place mixed sample over sensor
//
//			// Variable to control pumping B1
//			uint16_t Steps_PreB1 = 2000;// * gPump_Ratio;			// Number of pump steps to pull sample before buffer
//			uint16_t Steps_PostB1 = 4000;// * gPump_Ratio;		// Number of pump steps to pull sample after buffer
//			//			uint16_t Steps_follow_B1 = 600;// * gPump_Ratio;	// Steps to pump sample to get B1 mixture into mixing chamber
//			uint16_t Steps_center = 7350;// * gPump_Ratio;		// Steps to pump to place mixed sample over sensor
//
//			// Variables to control pumping C2
//			uint16_t Steps_Precond = 2000;// * gPump_Ratio;		// Number of pump steps to pull sample before buffer
//			uint16_t Steps_Postcond = 4000;// * gPump_Ratio;	// Number of pump steps to pull sample after buffer
//			uint16_t Steps_C2 = 264;// * gPump_Ratio;				// Steps to pump conditioner, should be constant
//			uint16_t Steps_follow_C2 = 600 - Steps_C2;// * gPump_Ratio;	// Change this number to change where in the mixing chamber the sample and conditioner are mixed
//			uint16_t Steps_center_B2 = 7350 + Steps_C2;// * gPump_Ratio;		// Steps to pump to place mixed sample over sensor
//
//			// Variables to control mixing B2
//			//			uint16_t Steps_sample_between_Cl = 10000 + Steps_follow_B1 + Steps_center;
//			uint16_t Steps_sample_between_Cl = 50000;
//			//			uint16_t Steps_back = (5500 + Steps_C2);// * gPump_Ratio; 		// Steps to pump mixed conditioner/sample back into valve before adding buffer, add C2 steps so pump starts at zero for buffer
//			uint16_t Steps_back = 6400;// * gPump_Ratio; 		// Steps to pump mixed conditioner/sample back into valve before adding buffer, add C2 steps so pump starts at zero for buffer
//			uint16_t Steps_forward = (5400 - Steps_B2);// * gPump_Ratio;	// Change this number to change where in mixing chamber the sample and buffer are mixed

			float PumpVol_air_bubble = 33.6;			// Volume uL of air bubble
			uint8_t PumpVol_Large_air_bubble = 84;		// Large air bubble on pre/post rinse
			float PumpVol_Solution = 33.6;				// Volume uL for solution between air bubbles
			float PumpVol_Clean = 117.6;				// Volume for clean plug
			float PumpVol_Clean_center = 125.86;		// Volume to center clean plug

			float PumpVol_Rinse = 100.8;		// Volume of prerinse after air bubbles
			float PumpVol_Sample = 134.4;		// Volume of sample after bubbles
			float PumpVol_plug = 141.29;		// Volume to center the measurement plugs on the sensors and reference.
			float PumpVol_plug_samp = 131.37;	// Volume to center sample plug for measurement
			uint16_t PumpVol_sample_rinse = 840;	// How much sample to pull before metering, large to clear air out of chip before metering buffers
//			uint16_t PumpVol_sample_rinse = 530;	// How much sample to pull before metering, this is the number used after milling out the turbidity chamber

			uint16_t PumpVol_Sample_Prime = 403;	// Volume to prime sample tube before initial rinse

			// Variables to control mixing
			uint16_t diffusion_time = 5000;				// Time to wait after oscillating in mixing chamber
			uint8_t mix_cycles = 10; //10;				// Number of forward backward pump cycles
			uint16_t diffusion_time_Cl = 0;				// Time to wait after oscillating in mixing chamber
			uint8_t mix_cycles_Cl = 5; //10;				// Number of forward backward pump cycles
			uint16_t Steps_cycles = 1500;// * gPump_Ratio;			// Steps to pump forward/backward

			// Priming B1
			float PumpVol_tube_bubble = 16.8;
			float PumpVol_tube_prime_buffers = 33.6;
//			float PumpVol_tube_prime = 16.8;

			uint16_t Test_Number = FindTestNumber() + 1; // Add 1 to go to next free spot in memory
			uint16_t Test_page = Find_Test_page(Test_Number);

			// Read step values from memory
			float PumpVol_Buffer = 13.22;	// Adjust this number for B1 and B2
//			float PumpVol_B2 = 13.22;	// Adjust this number for B2

			// Initialize floats to hold pump variables
			float PumpVolRev, Pump_Ratio;

			// Read from Tiva EEPROM the pump specs
			EEPROMRead((uint32_t *) &PumpVolRev, OFFSET_PUMP_VOL_PER_REV, 4);
			EEPROMRead((uint32_t *) &Pump_Ratio, OFFSET_PUMP_DEAD_SPOT, 4);

			if(PumpVolRev != PumpVolRev)
				PumpVolRev = 16.8;
			if(Pump_Ratio != Pump_Ratio)
				Pump_Ratio = 0.61;

			// Variable to control pumping T1
			uint16_t Steps_PreT1 = 3000;	// Number of pump steps to pull sample before buffer, leaving this as steps so pump starts metering in a known location
			float PumpVol_PostT1 = 117.6 - ((Steps_PreT1/1000) * PumpVolRev);	// Volume to pull sample after buffer
			float PumpVol_follow_T1 = 19.55;	// Volume to get T1 mixture into mixing chamber
			float PumpVol_center_T1 = 131.65;	// Volume to place mixed sample over sensor

			// Variable to control pumping B1
			uint16_t Steps_PreB1 = 2000;	// Number of pump steps to pull sample before buffer
			float PumpVol_PostB1 = 100.8 - ((Steps_PreB1/1000) * PumpVolRev);	// Number of pump steps to pull sample after buffer
			//			uint16_t Steps_follow_B1 = 600;// * gPump_Ratio;	// Steps to pump sample to get B1 mixture into mixing chamber
			float PumpVol_center = 119.67;		// Steps to pump to place mixed sample over sensor

			// Variables to control pumping C2
//			uint16_t Steps_Precond = 2000;// * gPump_Ratio;		// Number of pump steps to pull sample before buffer
//			uint16_t Steps_Postcond = 4000;// * gPump_Ratio;	// Number of pump steps to pull sample after buffer
			float PumpVol_C2 = 7.27;							// Volume of conditioner, should be constant
			float PumpVol_follow_C2 = 15.28 - PumpVol_C2;// * gPump_Ratio;	// Change this number to change where in the mixing chamber the sample and conditioner are mixed
			float PumpVol_center_B2 = 117.74 + PumpVol_C2;// * gPump_Ratio;		// Steps to pump to place mixed sample over sensor

			// Variables to control mixing B2
			//			uint16_t Steps_sample_between_Cl = 10000 + Steps_follow_B1 + Steps_center;
//			uint16_t PumpVol_sample_between_Cl = 840;
			//			uint16_t Steps_back = (5500 + Steps_C2);// * gPump_Ratio; 		// Steps to pump mixed conditioner/sample back into valve before adding buffer, add C2 steps so pump starts at zero for buffer
			float PumpVol_back = 100.8 + 15.28;	// Volume to pump mixed conditioner/sample back into valve before adding buffer, add C2 and follow steps so pump starts at zero for B2
			float PumpVol_forward = (100.8 + 15.28 - PumpVolRev - PumpVol_Buffer);		// Change this number to change where in mixing chamber the sample and buffer are mixed, pump forward the backward amount less a pump revolution and B2 volume


			// Variables to control pump speed
			// 6000 is slow pumping being used for everything else
			// Slowest speed is 8000, may be able to go slower but the function will need to be changed
			// Fastest speed is 3000, may work up to 2500 depends on the pump
			uint16_t Speed_Fast = 3000;
//			uint16_t Speed_mixing = 3000;
//			uint16_t Speed_placing = 3000;
//			uint16_t Speed_priming = 3000;
//			uint16_t Speed_ISE = 3000;
//			uint16_t Speed_C2 = 3000;
			uint16_t Speed_Metering = 6000;

			// Variable to know if sensor is currently in rinse solution, set to 1 when done pumping rinse, 0 when pumping
			// Used to know when abort happens if we need to pump in store
			uint8_t Sensor_in_rinse = 1;

			uint8_t i;

			uint16_t valve_delay = 1000;
			uint16_t valve_delay_after_air = 100; //1000;
			uint16_t valve_delay_metering = 500; // 2000;

			//
			// Collect solution data at beginning so conductivity and pH can be calculated throughout
			//
#ifdef SOLUTION_IN_STRUCT
			struct SolutionVals *Sols = FillSolutionStruct();

//			struct SolutionVals Sols;
//			FillSolutionStruct(&Sols);
#else
			float pH_EEP_Rinse, Ca_EEP_Rinse, TH_EEP_Rinse, NH4_EEP_Rinse, Cond_EEP_Rinse;
#ifdef SINGLE_POINT_H2_IN_CLEAN
			float pH_EEP_Clean;
#endif
			float Cond_EEP_Cal_2;
			float HCl_N;

			if(SOL_FROM_CART == 1)
			{
#ifdef CAL_2_RINSE
				// Rinse
				pH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_PH, 4));
				Ca_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_CA, 4));
				TH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_TH, 4));
				NH4_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_NH4, 4));
				Cond_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_COND, 4));

				pH_EEP_Clean = pH_EEP_Rinse;
#else
				// Rinse
				pH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_PH, 4));
				Ca_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_CA, 4));
				TH_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_TH, 4));
				NH4_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_NH4, 4));
				Cond_EEP_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_RINSE_COND, 4));

#ifdef SINGLE_POINT_H2_IN_CLEAN
				pH_EEP_Clean = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CLEAN_PH, 4));
#endif	// SINGLE_POINT_H2_IN_CLEAN

#endif	// CAL_2_RINSE

				// Cal 2
				Cond_EEP_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_CAL_2_COND, 4));

				// T1
				HCl_N = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_T1_HCL_N, 4));
			}
			else
			{
				// Rinse
				pH_EEP_Rinse = pH_Rinse_pouch;
				Ca_EEP_Rinse = Ca_Rinse_pouch;
				TH_EEP_Rinse = TH_Rinse_pouch;
				NH4_EEP_Rinse = NH4_Rinse_pouch;
				Cond_EEP_Rinse = Cond_Rinse_pouch;

				// Cal 2
				Cond_EEP_Cal_2 = Cond_Cal_2_pouch;

				HCl_N = HCl_N_pouch;
			}

			if(HCl_N != HCl_N)
				HCl_N = HCl_N_pouch;

			// Pull K T and IS values from memory
			float IS_RINSE, IS_CLEAN, IS_CAL_1, IS_CAL_2;
			float K_T_pH_Rinse, K_T_pH_Cal_1, K_T_pH_Cal_2, K_T_pH_Clean_Sq, K_T_pH_Clean_Ln;
			IS_RINSE = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_IS_RINSE, 4));
			if(SOL_FROM_CART == 1 && IS_RINSE == IS_RINSE)
			{
				IS_CLEAN = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_IS_CLEAN, 4));
				IS_CAL_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_IS_CAL_1, 4));
				IS_CAL_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_IS_CAL_2, 4));

				K_T_pH_Rinse = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_KT_RINSE, 4));
				K_T_pH_Cal_1 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_KT_CAL_1, 4));
				K_T_pH_Cal_2 = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_KT_CAL_2, 4));
				K_T_pH_Clean_Sq = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_KT_CLEAN_SQ, 4));
				K_T_pH_Clean_Ln = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_KT_CLEAN_LN, 4));
			}
			else
			{
				IS_RINSE = 0.0115;
				IS_CLEAN = 0.0187;
				IS_CAL_1 = 0.0321;
				IS_CAL_2 = 0.00335;

				K_T_pH_Rinse = -0.0129;
				K_T_pH_Cal_1 = -0.0025;
				K_T_pH_Cal_2 = -0.0243;
				K_T_pH_Clean_Sq = .00007;
				K_T_pH_Clean_Ln = -.0071;
			}
#endif

			if((gui32Error & ABORT_ERRORS) != 0)
				break;

			PrintTime();

//			if(STORE_AT_POTENTIAL)
//			{
//				// Let the working electrodes float when not measuring
//				IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
//				IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 0);
//
//				DACVoltageSet(0, 0, true);
//
//				// RE and CE floating
//				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);		// Leave RE floating
//				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);		// Leave CE floating
//			}


//			// Prime sample tube before test
//			if(START_IN_NITRIC)
//			{
//				DEBUG_PRINT(UARTprintf("Pumping Nitric Acid over amperometric arrays and letting sit for 10 minutes!\n");)
//
//				// Pump in rinse over amperometrics for cleaning
//				RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
//				FindPossitionZeroPump();
//				userDelay(valve_delay_after_air, 1);
//				RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);
//
//				if(BUBBLES_IN_TUBE)
//					PumpStepperRunStepSpeed_AbortReady(FW, Steps_tube_prime, Speed_ISE);
//				PumpStepperRunStepSpeed_AbortReady(FW, runSteps_Clean, Speed_ISE);
//				userDelay(valve_delay, 1);
//				RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
//				PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble + Steps_tube_bubble, Speed_ISE);
//				userDelay(valve_delay_after_air, 1);
//				RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);
//				PumpStepperRunStepSpeed_AbortReady(BW, Steps_tube_bubble, Speed_ISE);
//				userDelay(valve_delay, 1);
//				RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
//				PumpStepperRunStepSpeed_AbortReady(FW, runSteps_Clean_center, Speed_ISE);
//
//				SleepValve();
//
//				userDelay(600000, 1);
//
//				SetLED(BLUE_BUTTON_BLINK, 1);
//				BuzzerSound(400);
//				DEBUG_PRINT(UARTprintf("Put in sample and press button!\n");)
//				while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == GPIO_PIN_3);
//				while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == 0);
//				SetLED(BLUE_BUTTON_BLINK, 0);
//			}

			// Clear memory before starting record of test (do this so when we loop back through memory we don't have data filled in if it freezes/resets)
			for(i = 0; i < PAGES_FOR_TEST; i++)
			{
				uint8_t j;
				uint32_t Clear_mem = 0xFFFFFFFF;
				for(j = 0; j < 64; j++)
				{
					MemoryWrite(Test_page + i, (j * 4), 4, (uint8_t *) &Clear_mem);
				}
			}

#ifdef TESTING_MODE
			if(g_QCSolution >= 1 && g_QCSolution <= 5)
			{
				DEBUG_PRINT(UARTprintf("\nQC Solution %d\n", g_QCSolution);)
			}
#endif

			DEBUG_PRINT(UARTprintf("\nTest Number %d\n", Test_Number);)

			float T_Therm;
			if(1)	// Isolating the thermistor variables T_Therm_S, T_Therm_F, and Therm_Correction becasue this is the only place they are needed
			{
				float T_Therm_S = ReadThermistor();

				DEBUG_PRINT(UARTprintf("\nPriming %d uL of sample\n", PumpVol_Sample_Prime);)
				Sensor_in_rinse = 0;
				RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
				PumpVolume(FW, PumpVol_Sample_Prime, Speed_Fast, 1);

				float T_Therm_F = ReadThermistor();
				DEBUG_PRINT(UARTprintf("Therm Start and Final Temps:\t%d\t%d\tC*1000\n", (int) (T_Therm_S * 1000), (int) (T_Therm_F * 1000));)

				float Therm_Correction = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_THERM_CORRECTION, 4));

				if(Therm_Correction == Therm_Correction)
				{
					DEBUG_PRINT(UARTprintf("Using saved thermistor slope %d / 1000 to calculate thermistor reading\n", (int) (Therm_Correction * 1000));)
					T_Therm = T_Therm_S - (T_Therm_F - T_Therm_S) / Therm_Correction;	// Calculate thermistor correction based off saved slope
				}
				else
				{
					DEBUG_PRINT(UARTprintf("No therm slope found, using compiled slope!\n");)
					T_Therm = T_Therm_S - (T_Therm_F - T_Therm_S) / (-0.8);	// Calculate thermistor correction
				}

				DEBUG_PRINT(UARTprintf("Calculated Sample Temp:\t%d\n", (int) (T_Therm * 1000));)
				MemoryWrite(Test_page, OFFSET_START_THERM, 4, (uint8_t *) &T_Therm_S);
				MemoryWrite(Test_page, OFFSET_FINAL_THERM, 4, (uint8_t *) &T_Therm_F);
			}


			userDelay(valve_delay, 1);

			// Push air back into sample port before moving to next solution
			if(BUBBLES_IN_TUBE && ISEs.Config != PH_CL_CART)
			{
				RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
				PumpVolume(FW, PumpVol_tube_bubble * 4, Speed_Fast, 1);
				userDelay(valve_delay_after_air, 1);
				RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
				PumpVolume(BW, PumpVol_tube_bubble, Speed_Fast, 1);
				userDelay(valve_delay, 1);
			}

#ifdef TESTING_MODE
			uint64_t prime_clock = TimerValueGet64(WTIMER0_BASE);
			DEBUG_PRINT(UARTprintf("Time to prime: %d\n", (uint32_t) ((prime_clock - start_clock)/SysCtlClockGet()));)
			DEBUG_PRINT(UARTprintf("Total time elapsed: %d s, or %d m %d s\n", (uint32_t) ((prime_clock - start_clock)/SysCtlClockGet()), (uint32_t) ((prime_clock - start_clock)/SysCtlClockGet())/60,(uint32_t) ((prime_clock - start_clock)/SysCtlClockGet())%60);)
#endif
//			if(STRETCH_SENSOR && (gui32Error & ABORT_ERRORS) == 0)
//			{
//				if(CheckSensorSat())
//				{
//					DEBUG_PRINT(UARTprintf("Sensor has been sitting, pumping calibrants to wake it!\n");)
//
//					RunValveToPossition_Bidirectional_AbortReady(V_CAL_1, VALVE_STEPS_PER_POSITION);
//					PumpStepperRunStepSpeed_AbortReady(FW, 4000, Speed_ISE);
//					userDelay(valve_delay, 1);
//					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
//					PumpStepperRunStepSpeed_AbortReady(FW, runSteps_plug, Speed_ISE);
//					userDelay(5000, 1);	// Let solution sit over sensors for 5 seconds
//
//					RunValveToPossition_Bidirectional_AbortReady(V_CAL_2, VALVE_STEPS_PER_POSITION);
//					PumpStepperRunStepSpeed_AbortReady(FW, 4000, Speed_ISE);
//					userDelay(valve_delay, 1);
//					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
//					PumpStepperRunStepSpeed_AbortReady(FW, runSteps_plug, Speed_ISE);
//					userDelay(5000, 1);	// Let solution sit over sensors for 5 seconds
//				}
//			}

			//
			// Create record of test right before pumping anything out of pouches
			//
			uint16_t Cal_Number = FindCalNumber();
			uint16_t Cal_page = Find_Cal_page(Cal_Number);

			// Calculate how far off the reference has drifted
			int8_t Ref_drift = 0;
			if(REF_DRIFT != 0 && SATURATED_KCL_REF == 0)
				Ref_drift = Calculate_Ref_Drift(SATURATED_KCL_REF, 25);

//			uint16_t Test_page = Find_Test_page(Test_Number);

			// Save test data to cartridge memory
			MemoryWrite(Test_page, OFFSET_TEST_NUMBER, 2, (uint8_t *) &Test_Number);
			MemoryWrite(Test_page, OFFSET_TEST_DATE, 4, (pui8SysStatus));
			MemoryWrite(Test_page, OFFSET_TEST_TIME, 3, (pui8SysStatus + 4));
			MemoryWrite(Test_page, OFFSET_TEST_USER_NAME, 4, (pui8SysStatus + 7));
//			MemoryWrite(Test_page, OFFSET_TEST_GPS, 8, (uint8_t *) (pui8SysStatus + 27));
			MemoryWrite(Test_page, OFFSET_TEST_LOCATION, 4, (pui8SysStatus + 35));
			MemoryWrite(Test_page, OFFSET_TEST_CAL, 1, (uint8_t *) &Cal_Number);
			MemoryWrite(Test_page, OFFSET_TEST_ERROR, 4, (uint8_t *) &gui32Error);

			if(1)	// Putting variables we only need once in a conditional to save stack space
			{
				float CPU_Temperature = GetCPUTemp();
				uint8_t Zero = 0;
				MemoryWrite(Test_page, OFFSET_TEST_ZERO, 1, &Zero);
				MemoryWrite(Test_page, OFFSET_TEST_CPU_TEMP, 4, (uint8_t *) &CPU_Temperature);

				uint8_t Device_Serial[8];
				EEPROMRead((uint32_t *) Device_Serial, OFFSET_SERIAL_NUMBER, 8);
				MemoryWrite(Test_page, OFFSET_TEST_DEVICE_SERIAL, 7, Device_Serial);

				MemoryWrite(Test_page, OFFSET_TEST_DATA_ZERO, 1, &Zero);
				MemoryWrite(Test_page, OFFSET_RAW_TEST_ZERO, 1, &Zero);
			}

//			MemoryWrite(Test_page, OFFSET_START_THERM, 4, (uint8_t *) &T_Therm_S);
//			MemoryWrite(Test_page, OFFSET_FINAL_THERM, 4, (uint8_t *) &T_Therm_F);
			MemoryWrite(Test_page, OFFSET_TEST_T_THERM, 4, (uint8_t *) &T_Therm);

			update_Test(Test_Number);

			//
			// Flow Chart Measurement Teal section, pre-rinse
			//
			float ISE_E_Rinse[10] = {0,0,0,0,0,0,0,0,0,0};
			float *pH_H2_E_Rinse = &ISE_E_Rinse[ISEs.pH_H2.index];
			float *pH_Cr_E_Rinse = &ISE_E_Rinse[ISEs.pH_Cr.index];
			float *TH_E_Rinse = &ISE_E_Rinse[ISEs.TH.index];
			float *NH4_E_Rinse = &ISE_E_Rinse[ISEs.NH4.index];
			float *Ca_E_Rinse = &ISE_E_Rinse[ISEs.Ca.index];
			float T_Rinse;
			float Conductivity_Rinse;
			//			uint16_t time_to_wait;
			//			uint16_t cycle;
#ifndef STRAIGHT_TO_CL
			if(ISEs.Config != PH_CL_CART)
			{
				uint8_t Repeat_index = 0;
				for(Repeat_index = 0; Repeat_index < REPEAT_PRERINSE; Repeat_index++)
				{
					PrintTime();
					if((gui32Error & ABORT_ERRORS) == 0)
					{
						update_Status(STATUS_TEST, OPERATION_TEST_RINSE);
#ifdef CAL_2_RINSE
						DEBUG_PRINT(UARTprintf("Pumping Cal 2 as prerinse... \n");)
#else
						DEBUG_PRINT(UARTprintf("Pumping prerinse... \n");)
#endif

						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
						FindPossitionZeroPump();
						uint16_t PumpSpeed = Speed_Fast;
						for (i = 0; i < Number_of_bubbles_Prerinse; i++) // Loop over air/solution cycle 3 times for single solution
						{
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
							PumpVolume(FW, PumpVol_air_bubble, PumpSpeed, 1);
							if(i == 2)// On the 3rd bubble I want to decrease pump speed before starting the large bubble so the first plug of rinse is moving slower when it hits the ISEs
								PumpSpeed = Speed_Metering;
							if(i == (Number_of_bubbles_Prerinse - 1))
								PumpVolume(FW, PumpVol_Large_air_bubble, PumpSpeed, 1);
							userDelay(valve_delay_after_air, 1);
#ifdef CAL_2_RINSE
							RunValveToPossition_Bidirectional_AbortReady(V_CAL_2, VALVE_STEPS_PER_POSITION);
#else
							RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);
#endif

							if(i == 0 && BUBBLES_IN_TUBE)
								PumpVolume(FW, PumpVol_tube_bubble, PumpSpeed, 1);
							PumpVolume(FW, PumpVol_Solution, PumpSpeed, 1);
							if(i != (Number_of_bubbles_Prerinse - 1))
								userDelay(valve_delay, 1);
						}
						PumpVolume(FW, PumpVol_Rinse, PumpSpeed, 1);
						userDelay(valve_delay, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						PumpVolume(FW, PumpVol_plug, PumpSpeed, 1);

						SleepValve();

						if((gui32Error & ABORT_ERRORS) == 0)
							Sensor_in_rinse = 1;
					}

					CollectISEmV(ISE_E_Rinse, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);
#ifdef RESET_ELECTRONICS_READ
					if(RESET_ELECTRONICS_READ)
					{
						AnalogOff();
						userDelay(20000, 1);
						InitAnalog();

						CollectISEmV(ISE_E_Rinse, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);
					}
#endif

					T_Rinse = MeasureTemperature(1);

					// Set RE and CE floating and close RE/CE loop
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);


//					if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
//						Conductivity_Rinse = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_2, 0);
//					else
//						Conductivity_Rinse = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);
//
//					DEBUG_PRINT(UARTprintf("Rinse Conductivity Measured at: %d\n", (int) (Conductivity_Rinse * 1000));)
//					if(ISEs.Config == PH_CL_CART)
//						DEBUG_PRINT(UARTprintf("Rinse temp and corrected conductivity: =%d/1000\t=%d/1000\n", (int) (T_Rinse * 1000), (int) ((Conductivity_Rinse / (1 + Sols->Clean_Cond_TComp*(T_Rinse - 25))) * 1000));)
//					else
//						DEBUG_PRINT(UARTprintf("Rinse temp and corrected conductivity: =%d/1000\t=%d/1000\n", (int) (T_Rinse * 1000), (int) ((Conductivity_Rinse / (1 + Sols->Rinse_Cond_TComp*(T_Rinse - 25))) * 1000));)
//					DEBUG_PRINT(UARTprintf("Rinse Conductivity saved on memory: %d\n", (int) (Sols->Cond_EEP_Rinse * 1000));)

					MemoryWrite(Test_page, OFFSET_RAW_T_RINSE, 4, (uint8_t *) &T_Rinse);
					for(i = 0; i < 10; i++)
						MemoryWrite(Test_page, OFFSET_RAW_ISE_1_RINSE + (i * 4), 4, (uint8_t *) &ISE_E_Rinse[i]);

					update_Test(Test_Number);

					// Push air back into rinse port before moving to next solution
					if(BUBBLES_IN_TUBE)
					{
#ifdef CAL_2_RINSE
						RunValveToPossition_Bidirectional_AbortReady(V_CAL_2, VALVE_STEPS_PER_POSITION);
#else
						RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);
#endif
						PumpVolume(BW, PumpVol_tube_bubble, Speed_Fast, 1);
						userDelay(valve_delay, 1);
					}
				}
			}
#endif	// STRAIGHT_TO_CL

#ifdef TESTING_MODE
			uint64_t rinse_clock = TimerValueGet64(WTIMER0_BASE);
			DEBUG_PRINT(UARTprintf("Time to rinse: %d\n", (uint32_t) ((rinse_clock - prime_clock)/SysCtlClockGet()));)
			DEBUG_PRINT(UARTprintf("Total time elapsed: %d s, or %d m %d s\n", (uint32_t) ((rinse_clock - start_clock)/SysCtlClockGet()), (uint32_t) ((rinse_clock - start_clock)/SysCtlClockGet())/60,(uint32_t) ((rinse_clock - start_clock)/SysCtlClockGet())%60);)
#endif

			//
			// Flow chart measurement green section, sample
			//
			PrintTime();
			uint32_t Cal_Status = *((uint32_t *) MemoryRead(Cal_page, OFFSET_CAL_STATUS, 4));
			uint8_t ISE_Cal_Status[10];
			uint8_t Cond_Cal_Status = (Cal_Status >> 11) & 1;
//			uint8_t pH_Cr_passed = 0;
			for(i = 0; i < 10; i++)
				ISE_Cal_Status[i] = (Cal_Status >> (i + 1)) & 1;
//			for(i = 0; i < ISEs.pH_Cr.size; i++)
//				if(ISE_Cal_Status[ISEs.pH_Cr.index + i] == 1)
//					pH_Cr_passed++;

			float T_Samp = T_ASSUME;
			float ISE_E_Samp[10] = {0,0,0,0,0,0,0,0,0,0};
			float *pH_H2_E_Samp = &ISE_E_Samp[ISEs.pH_H2.index];
			float *pH_Cr_E_Samp = &ISE_E_Samp[ISEs.pH_Cr.index];
			float *TH_E_Samp = &ISE_E_Samp[ISEs.TH.index];
			float *NH4_E_Samp = &ISE_E_Samp[ISEs.NH4.index];
			float *Ca_E_Samp = &ISE_E_Samp[ISEs.Ca.index];
			float ORP = 0;
			float Conductivity = 0;

			uint8_t * ptr_Last_cal_passed = MemoryRead(Cal_page, OFFSET_PH_1_LAST_P_CAL, 10);
			uint8_t Last_cal_passed[10];
			memcpy(Last_cal_passed, ptr_Last_cal_passed, 10);
			for(i = 0; i < 10; i++)
				if(Last_cal_passed[i] == 0xFF || Last_cal_passed[i] == 0)
					Last_cal_passed[i] = Cal_Number;

			//			float pH_EEP_Slope[3], T_EEP_Cal;
			//			float Ca_EEP_Slope[2];
			//			float NH4_EEP_Slope[3], TH_EEP_Slope[2];
			//			float T_EEP_Cal;
			float ISE_EEP_Slope[10] = {0,0,0,0,0,0,0,0,0,0};
			float *pH_H2_EEP_Slope = &ISE_EEP_Slope[ISEs.pH_H2.index];
			float *pH_Cr_EEP_Slope = &ISE_EEP_Slope[ISEs.pH_Cr.index];
			float *TH_EEP_Slope = &ISE_EEP_Slope[ISEs.TH.index];
			//			float Mg_EEP_Slope[2];
			float *NH4_EEP_Slope = &ISE_EEP_Slope[ISEs.NH4.index];
			float *Ca_EEP_Slope = &ISE_EEP_Slope[ISEs.Ca.index];

			for(i = 0; i < 10; i++)
				ISE_EEP_Slope[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i]), OFFSET_ISE_1_SLOPE + (i * 4), 4));

			float T_EEP_Cal = Build_float(MemoryRead(Cal_page, OFFSET_T_CAL, 4));
			float IS;
			float ISE_Reading[10] = {0,0,0,0,0,0,0,0,0,0};
			float *pH_H2_Samp = &ISE_Reading[ISEs.pH_H2.index];
			float *pH_Cr_Samp = &ISE_Reading[ISEs.pH_Cr.index];
			float *TH_corr = &ISE_Reading[ISEs.TH.index];
			float *NH4_NH3_N_Free = &ISE_Reading[ISEs.NH4.index];
			float *Ca_Hardness = &ISE_Reading[ISEs.Ca.index];
			float NH4_Ammonium;	// Define free ammonia here because it is used in calculations with chlorine later
			float log_K_Ca_Mg = LOG_K_CA_MG;
			float T_RS;
			float pH_TCor_Rinse;
			float pH_Cr_Samp_RS;

//			float log_K_Ca_Mg_Nick[2];

			// Save chosen sensors bit-wise, Alk,Alk,NH4,NH4,TH,Ca,pH,pH
			//			uint8_t ui8Chosen_Sensors = PS_Chosen_pH | (PS_Chosen_Ca << 2) | (PS_Chosen_TH << 3) | (PS_Chosen_NH4 << 4);
			uint8_t T_Chosen_pH = 0, T_Chosen_Ca = 0, T_Chosen_TH = 0, T_Chosen_NH4 = 0, T_Chosen_Alk = 0;
#ifdef PRINT_UART
			float TH_iterated[2];
			uint8_t T_Chosen_TH_RR = 0;
#endif
			uint8_t ui8TChosen_Sensors;

			uint8_t Repeat_index = 0;

			for(Repeat_index = 0; Repeat_index < REPEAT_SAMP; Repeat_index++)
			{
#ifndef STRAIGHT_TO_CL
				if((gui32Error & ABORT_ERRORS) == 0)
				{
					update_Status(STATUS_TEST, OPERATION_SAMPLE);
					DEBUG_PRINT(UARTprintf("Pumping sample... \n");)

					Sensor_in_rinse = 0;

#if defined BACK_AND_FORTH || defined DIFFUSION_RINSE
					if(BACK_AND_FORTH)
					{
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
						FindPossitionZeroPump();
						for (i = 0; i < Number_of_bubbles_samp - 1; i++) // Loop over air/solution cycle 3 times for single solution
						{
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
							PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 1);
							if(i == (Number_of_bubbles_samp - 1))
								PumpVolume(FW, PumpVol_Large_air_bubble, Speed_Fast, 1);
							userDelay(valve_delay_after_air, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
							if(i == 0 && BUBBLES_IN_TUBE)
								PumpVolume(FW, PumpVol_tube_bubble, Speed_Fast, 1);
							PumpVolume(FW, PumpVol_Solution, Speed_Fast, 1);
							if(i != (Number_of_bubbles_samp - 1))
								userDelay(valve_delay, 1);
						}

						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						PumpVolume(FW, PumpVol_Large_air_bubble + PumpVol_air_bubble, Speed_Fast, 1);
						userDelay(valve_delay_after_air, 1);

						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);

						PumpStepperMix(BW, 8000, Speed_Fast, 10);

						PumpVolume(FW, PumpVol_Sample + PumpVol_Solution, Speed_Fast, 1);
						userDelay(valve_delay, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						PumpVolume(FW, PumpVol_plug_samp, Speed_Fast, 1);
						SleepValve();
					}
					else if(DIFFUSION_RINSE)
					{
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
						FindPossitionZeroPump();
						for (i = 0; i < Number_of_bubbles_samp; i++) // Loop over air/solution cycle 3 times for single solution
						{
							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
							PumpVolume(FW, PumpVol_Sample + 134.4, Speed_Fast, 1);
							userDelay(valve_delay, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
							PumpVolume(FW, PumpVol_plug_samp, Speed_Fast, 1);
							userDelay(10000, 1);	// Let solution sit over sensors for 5 seconds
						}

						SleepValve();
					}
					else
#endif
					{
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
						FindPossitionZeroPump();
						uint16_t PumpSpeed = Speed_Fast;
						for (i = 0; i < Number_of_bubbles_samp; i++) // Loop over air/solution cycle 3 times for single solution
						{
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
							PumpVolume(FW, PumpVol_air_bubble, PumpSpeed, 1);
							if(i == 2)
								PumpSpeed = Speed_Metering;
							if(i == (Number_of_bubbles_samp - 1))
								PumpVolume(FW, PumpVol_Large_air_bubble, PumpSpeed, 1);
							userDelay(valve_delay_after_air, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
							if(i == 0 && BUBBLES_IN_TUBE)
								PumpVolume(FW, PumpVol_tube_bubble, PumpSpeed, 1);
							PumpVolume(FW, PumpVol_Solution, PumpSpeed, 1);
							if(i != (Number_of_bubbles_samp - 1))
								userDelay(valve_delay, 1);
						}

						if(MEAUSURE_WHILE_PUMPING || PUMP_THEN_MEASURE)	// If we measure while pumping stay at the sample port so we can keep pumping through
						{
							PumpVolume(FW, PumpVol_Sample + PumpVol_plug_samp, PumpSpeed, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
						}
						else
						{
							PumpVolume(FW, PumpVol_Sample, PumpSpeed, 1);
							userDelay(valve_delay, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
							PumpVolume(FW, PumpVol_plug_samp, PumpSpeed, 1);
						}

						SleepValve();
					}
				}


//				if(MEAUSURE_WHILE_PUMPING)
//				{
//					CollectISEmV_WhilePumping(FW, 6000, ISE_E_Samp, 0xFFFF, 60, PRINT_ISE_TIME_DATA, &ISEs);
////					CollectISEmV(ISE_E_Samp, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);
//				}
//				else if(PUMP_THEN_MEASURE)
				if(PUMP_THEN_MEASURE)
				{

					// GND RE for ISEs
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 0);		// GND RE
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);		// Leave CE floating

					ConnectMemory(0);

					float ISE_E_Temp[10];
					PumpStepperRunTimeSpeed_AbortReady(FW, 15, 3000);
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					CollectISEmV(ISE_E_Temp, 0xFFFF, 2, PRINT_ISE_TIME_DATA, &ISEs);
					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Always start with air purge

					PumpStepperRunTimeSpeed_AbortReady(FW, 10, 3000);
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					CollectISEmV(ISE_E_Samp, 0xFFFF, 2, PRINT_ISE_TIME_DATA, &ISEs);

					uint8_t Stable = 1;
					for(i = 0; i < ISEs.pH_Cr.size; i++)
					{
						if(ISE_Cal_Status[ISEs.pH_Cr.index + i] && abs_val(ISE_E_Temp[ISEs.pH_Cr.index + i] - ISE_E_Samp[ISEs.pH_Cr.index + i]) > 1)	// If pH Cr sensor passed calibration and the mV drifted more than 1 mV from 15 seconds to 25 seconds say unstable
							Stable = 0;
					}

					uint8_t RepeatIndex;
					for(RepeatIndex = 1; RepeatIndex < PUMP_THEN_MEASURE_REPEAT; RepeatIndex++)
					{
						if(Stable == 0)
						{
							// Set RE and CE floating and close RE/CE loop
							IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
							IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);

							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
							PumpVolume(FW, PumpVol_air_bubble + PumpVol_Large_air_bubble, Speed_Fast, 1);
							userDelay(valve_delay_after_air, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
							PumpVolume(FW, PumpVol_Solution + PumpVol_Sample + PumpVol_plug_samp, Speed_Metering, 1);

							// GND RE for ISEs
							IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 0);		// GND RE
							IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);		// Leave CE floating

							ConnectMemory(0);

							float ISE_E_Temp[10];
							PumpStepperRunTimeSpeed_AbortReady(FW, 15, 3000);
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
							CollectISEmV(ISE_E_Temp, 0xFFFF, 2, PRINT_ISE_TIME_DATA, &ISEs);

							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
							PumpStepperRunTimeSpeed_AbortReady(FW, 10, 3000);
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
							CollectISEmV(ISE_E_Samp, 0xFFFF, 2, PRINT_ISE_TIME_DATA, &ISEs);

							Stable = 1;
							for(i = 0; i < ISEs.pH_Cr.size; i++)
							{
								if(ISE_Cal_Status[ISEs.pH_Cr.index + i] && abs_val(ISE_E_Temp[ISEs.pH_Cr.index + i] - ISE_E_Samp[ISEs.pH_Cr.index + i]) > 1)	// If pH Cr sensor passed calibration and the mV drifted more than 1 mV from 15 seconds to 25 seconds say unstable
									Stable = 0;
							}
						}
					}

					ConnectMemory(1);
				}
				else
					CollectISEmV(ISE_E_Samp, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);

				if(gABoard >= ARV1_0B)
					ORP = (2 * ADCReadAvg(ORP_CH, ORP_ADC, 10) - 3000)/1 + 209;
				else
					ORP = (2 * ADCReadAvg(ORP_CH, ORP_ADC, 10) - 3000)/2 + 209;
#ifdef RESET_ELECTRONICS_READ
				if(RESET_ELECTRONICS_READ)
				{
					AnalogOff();
					userDelay(20000, 1);
					InitAnalog();

					CollectISEmV(ISE_E_Samp, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);
				}
#endif

				ConnectMemory(1);

				T_Samp = MeasureTemperature(1);
				if(REF_DRIFT != 0 && SATURATED_KCL_REF != 0)
					ORP += Calculate_Ref_Drift(SATURATED_KCL_REF, T_Samp);
				else
					ORP += Ref_drift;

#ifndef COND_SOLUTION_STRUCT
				if(Sols->pH_EEP_Cal_2 < 9) // This is Cal 3, not Cal 2
				{
					if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
						Conductivity = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_1, Test_Number);
					else
						Conductivity = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_1, Test_Number);
				}
				else
				{
					if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
						Conductivity = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_2, Test_Number);
					else
						Conductivity = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, Test_Number);
				}
#else
				Conductivity = MeasureConductivity(Sols, Test_Number);
#endif	// COND_SOLUTION_STRUCT

				DEBUG_PRINT(UARTprintf("Conductivity read: =%d/1000\n", (int) (Conductivity * 1000));)
				DEBUG_PRINT(UARTprintf("Temp and corrected conductivity:\t=%d/1000\t=%d/1000\n", (int) (T_Samp * 1000), (int) ((Conductivity / (1 + COND_TCOMP_SAMP*(T_Samp - 25))) * 1000));)
				if(Conductivity < 25)
				{
					DEBUG_PRINT(UARTprintf("Setting conductivity to 25 for calculations!\n");)
					Conductivity = 25;
				}

#ifdef DRIVE_COUNTER_TIME
				if(DRIVE_COUNTER_TIME != 0)
				{
					if(DRIVE_COUNTER_VOLTAGE != 0)
					{
						IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 0);
						IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
						IO_Ext_Set(IO_EXT2_ADDR, 3, COUNTER_EL_DRIVE, 1);
						DACVoltageSet(6, DRIVE_COUNTER_VOLTAGE, true);	// Set the DAC COUNTER_V_DRIVE channel
					}

					userDelay(DRIVE_COUNTER_TIME * 1000, 1);

					DACVoltageSet(6, 0, true);	// Set the DAC COUNTER_V_DRIVE channel
					IO_Ext_Set(IO_EXT2_ADDR, 3, COUNTER_EL_DRIVE, 0);

					// Set RE and CE floating and close RE/CE loop
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);

					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
					PumpVolume(FW, PumpVol_Sample + PumpVol_Solution, Speed_Fast, 1);
					userDelay(valve_delay, 1);
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					PumpVolume(FW, PumpVol_plug_samp, Speed_Fast, 1);
					SleepValve();

					CollectISEmV(ISE_E_Samp, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);
					if(gABoard >= ARV1_0B)
						ORP = (2 * ADCReadAvg(ORP_CH, ORP_ADC, 10) - 3000)/1 + 209;
					else
						ORP = (2 * ADCReadAvg(ORP_CH, ORP_ADC, 10) - 3000)/2 + 209;
#ifdef RESET_ELECTRONICS_READ
					if(RESET_ELECTRONICS_READ)
					{
						AnalogOff();
						userDelay(20000, 1);
						InitAnalog();

						CollectISEmV(ISE_E_Samp, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);
					}
#endif

					ConnectMemory(1);

					T_Samp = MeasureTemperature(1);
					if(REF_DRIFT != 0 && SATURATED_KCL_REF != 0)
						ORP += Calculate_Ref_Drift(SATURATED_KCL_REF, T_Samp);
					else
						ORP += Ref_drift;

					Conductivity = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, Test_Number);
					DEBUG_PRINT(UARTprintf("Conductivity read: =%d/1000\n", (int) (Conductivity * 1000));)
					DEBUG_PRINT(UARTprintf("Temp and corrected conductivity:\t=%d/1000\t=%d/1000\n", (int) (T_Samp * 1000), (int) ((Conductivity / (1 + COND_TCOMP_SAMP*(T_Samp - 25))) * 1000));)
					if(Conductivity < 50)
					{
						DEBUG_PRINT(UARTprintf("Setting conductivity to 50 for calculations!\n");)
						Conductivity = 50;
					}
				}
#endif	// DRIVE_COUNTER_TIME

				// Set RE and CE floating and close RE/CE loop
				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);

				MemoryWrite(Test_page, OFFSET_TEST_TEMP, 4, (uint8_t *) &T_Samp);
				for(i = 0; i < 10; i++)
					MemoryWrite(Test_page, OFFSET_RAW_ISE_1_SAMP + (i * 4), 4, (uint8_t *) &ISE_E_Samp[i]);
				MemoryWrite(Test_page, OFFSET_TEST_ORP, 4, (uint8_t *) &ORP);

				update_Test(Test_Number);

				//				if((gui32Error & ABORT_ERRORS) == 0)
				//				{
				//					DEBUG_PRINT(UARTprintf("Running a dummy Cl read/clean in sample!\n");)
				//					ConnectMemory(0);
				//
				//					// Read ADC here to make sure it is working, I've seen ADC have problem during Cl read causing analog board to reset which threw off
				//					// Cl reading, by reading here hopefully we catch the problem and fix it before doing anything to the amperometric arrays
				//					ADCReadAvg(0, ADC4_CS_B, 5);
				//
				//					// Turn on short and off parallel resistor to allow large current flows
				//					IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 1);	// Parallel switch must be on with short switch to work
				//					IO_Ext_Set(IO_EXT2_ADDR, 2, WORK_EL_SHORT, 1);
				//					if(HIGH_RANGE)
				//						IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_MID_CURRENT, 1);
				//
				//					// Set reference for amperometric mode
				//					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
				//					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);
				//
				//					// Connect all electrodes together for measuring
				//					IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 1);
				//					IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);
				//
				//					DACVoltageSet(0, 750, true);
				//
				//					userDelay(3000, 1);	// Delay 3 seconds to let large current flow through before switching in reading resistor
				//
				//					float Cl_nA_Samp;
				//					IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 0);
				//					IO_Ext_Set(IO_EXT2_ADDR, 2, WORK_EL_SHORT, 0);	// Turn off short switch
				//					if(HIGH_RANGE)
				//						Cl_nA_Samp = *(CurrentTimeRead(0, ADC4_CS_B, 12, (int) 750, 2, .005) + 1);	// nA
				//					else
				//						Cl_nA_Samp = *(CurrentTimeRead(0, ADC4_CS_B, 12, (int) 750, 0, .005) + 1);	// nA
				//
				//					// Let the working electrodes float when not measuring
				//					IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
				//					IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 0);
				//					if(HIGH_RANGE)
				//						IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_MID_CURRENT, 0);
				//					DACVoltageSet(0, 0, true);
				//
				//					ConnectMemory(1);
				//
				//					DEBUG_PRINT(UARTprintf("Dummy read raw: %d nA * 1000\n", (int) (Cl_nA_Samp * 1000));)
				//				}

				//
				// Calculations for ISEs, done before alkalinity so we can make guess on how much T1 to pump in
				//
				ConnectMemory(1);

				T_RS = (T_Rinse + T_Samp) / 2;
#ifdef CAL_2_RINSE
				pH_TCor_Rinse = Calc_pH_TCor(Sols->pH_EEP_Cal_2, T_RS, 25, 0, Sols->K_T_pH_Cal_2);
#else
				pH_TCor_Rinse = Calc_pH_TCor(Sols->pH_EEP_Rinse, T_RS, 25, 0, Sols->K_T_pH_Rinse);
#endif


				//			float pH_TCor_Rinse = pH_EEP_Rinse + K_T_pH_Rinse * (T_RS - 25);	// Temperature corrected pH for Rinse

				//
				// ISE Calculations, moved here so pH, Ca, and TH can be used for Cl and Alkalinity mixings
				//
				//			uint8_t pH_Cal_Status[3] = {(Cal_Status >> 1) & 1, (Cal_Status >> 2) & 1, (Cal_Status >> 28) & 1};
				DEBUG_PRINT(UARTprintf("\tLast Passed Cal\tSlope\n");)
				for(i = 0; i < ISEs.pH_H2.size; i++)
					{DEBUG_PRINT(UARTprintf("pH H2 %d\t%d\t%d\n", i + 1, Last_cal_passed[i + ISEs.pH_H2.index], (int) (pH_H2_EEP_Slope[i] * 1000));)}
				for(i = 0; i < ISEs.pH_Cr.size; i++)
					{DEBUG_PRINT(UARTprintf("pH Cr %d\t%d\t%d\n", i + 1, Last_cal_passed[i + ISEs.pH_Cr.index], (int) (pH_Cr_EEP_Slope[i] * 1000));)}
				for(i = 0; i < ISEs.TH.size; i++)
					{DEBUG_PRINT(UARTprintf("TH %d\t%d\t%d\n", i + 1, Last_cal_passed[i + ISEs.TH.index], (int) (TH_EEP_Slope[i] * 1000));)}
				for(i = 0; i < ISEs.NH4.size; i++)
					{DEBUG_PRINT(UARTprintf("NH4 %d\t%d\t%d\n", i + 1, Last_cal_passed[i + ISEs.NH4.index], (int) (NH4_EEP_Slope[i] * 1000));)}
				for(i = 0; i < ISEs.Ca.size; i++)
					{DEBUG_PRINT(UARTprintf("Ca %d\t%d\t%d\n", i + 1, Last_cal_passed[i + ISEs.Ca.index], (int) (Ca_EEP_Slope[i] * 1000));)}
				DEBUG_PRINT(UARTprintf("\n");)

				if((gui32Error & ABORT_ERRORS) == 0 && ISEs.Config != PH_CL_CART)
				{
					DEBUG_PRINT(UARTprintf("Therm Temp: %d\n", (int) (T_Therm * 1000));)
					DEBUG_PRINT(UARTprintf("Sensor Temp: %d\n", (int) (T_Samp * 1000));)

					//
					// pH H2 Measurement
					//
					for(i = 0; i < ISEs.pH_H2.size; i++)
					{
						float pH_H2_Slope_RST = pH_H2_EEP_Slope[i] * (T_RS + 273.0) / (T_EEP_Cal + 273.0);	// Temperature corrected slope
						// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
						float pH_H2_Samp_RS = pH_TCor_Rinse + ((pH_H2_E_Samp[i] - pH_H2_E_Rinse[i]) / pH_H2_Slope_RST); // pH of sample
						if(T_Therm > 2 && T_Therm < 50)
						{
							pH_H2_Samp[i] = Calc_pH_TCor(pH_H2_Samp_RS, T_Therm, T_RS, K_T_pH_Samp_Sq, K_T_pH_Samp_Ln);
							//						pH_H2_Samp[i] = pH_H2_Samp_RS + (K_T_pH_Samp_Sq * (pow(T_Therm, 2) - pow(T_RS, 2)) + K_T_pH_Samp_Ln * (T_Therm - T_RS));//+ K_T_pH_Samp * (T_Therm - T_RS);
							DEBUG_PRINT(UARTprintf("pH H2 %d, Uncorrected: %d, Corrected: %d\n", i + 1, (int) (pH_H2_Samp_RS * 1000), (int) (pH_H2_Samp[i] * 1000));)
						}
						else
						{
							//						DEBUG_PRINT(UARTprintf("Thermistor temperature outside of range, not adjusting pH!\n");)
							pH_H2_Samp[i] = pH_H2_Samp_RS;
							gui32Error |= THERMISTOR_FAILED;
						}
					}

					//
					// pH Cr Measurement
					//
					//				float pH_Slope_SampT[3]; //pH_E_Samp_TCor[3];//, pH_Samp[2];
					for(i = 0; i < ISEs.pH_Cr.size; i++)
					{
						float pH_Cr_Slope_RST = pH_Cr_EEP_Slope[i] * (T_RS + 273.0) / (T_EEP_Cal + 273.0);	// Temperature corrected slope
						// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
						pH_Cr_Samp_RS = pH_TCor_Rinse + ((pH_Cr_E_Samp[i] - pH_Cr_E_Rinse[i]) / pH_Cr_Slope_RST); // pH of sample
						if(T_Therm > 2 && T_Therm < 50)
						{
							pH_Cr_Samp[i] = Calc_pH_TCor(pH_Cr_Samp_RS, T_Therm, T_RS, K_T_pH_Samp_Sq, K_T_pH_Samp_Ln);
							DEBUG_PRINT(UARTprintf("pH Cr %d, Uncorrected: %d, Corrected: %d\n", i + 1, (int) (pH_Cr_Samp_RS * 1000), (int) (pH_Cr_Samp[i] * 1000));)
						}
						else
						{
							//						DEBUG_PRINT(UARTprintf("Thermistor temperature outside of range, not adjusting pH!\n\n");)
							pH_Cr_Samp[i] = pH_Cr_Samp_RS;
							gui32Error |= THERMISTOR_FAILED;
						}
					}

					if(T_Therm < 2 || T_Therm > 50)
						{DEBUG_PRINT(UARTprintf("Thermistor temperature outside of range, not adjusting pH!\n");)}
					DEBUG_PRINT(UARTprintf("\n");)

#ifndef UNIVERSAL_PICKING_FUNCTION
					if(ISEs.Config == PH_H2_CART)
						T_Chosen_pH = Choose_pH_Sensor_pHDie(Cal_Number, ISE_Reading);
					else
						T_Chosen_pH = Choose_pH_Sensor(Cal_Number, pH_Cr_Samp, pH_Cr_E_Rinse, T_RS, ISEs, Sols);
#else	// UNIVERSAL_PICKING_FUNCTION
					T_Chosen_pH = Choose_Sensor(Cal_Number, pH_Cr_Samp, pH_Cr_E_Rinse, T_Rinse, ISEs.pH_Cr, Sols);
#endif	// UNIVERSAL_PICKING_FUNCTION



					// Recalculate the uncorrected pH for the chosen sensor so it can be used in NH4 math
					pH_Cr_Samp_RS = pH_TCor_Rinse + ((pH_Cr_E_Samp[T_Chosen_pH] - pH_Cr_E_Rinse[T_Chosen_pH]) / (pH_Cr_EEP_Slope[T_Chosen_pH] * (T_RS + 273.0) / (T_EEP_Cal + 273.0))); // pH of sample

					if(ISE_Cal_Status[ISEs.pH_Cr.index + T_Chosen_pH] && ISEs.pH_Cr.size > 0)	// Check that the chosen sensor passed calibration before reporting a number
						MemoryWrite(Test_page, OFFSET_TEST_PH, 4, (uint8_t *) &pH_Cr_Samp[T_Chosen_pH]);

					//
					// Ca Measurement
					//
					//
					// Conductivity Temperature Correction
					// 3/20/2023: Cond temp correction to before we calculate the ionic strength because the model is based on conductivities at 25C
					//
					// Perform temperature correction here after calculations for ISEs so we are using the conductivity at temperature, not the adjusted conductivity
					Conductivity /= (1 + COND_TCOMP_SAMP*(T_Samp - 25));

#ifndef TESTING_MODE
					if(Conductivity < 0)
						Conductivity = 0;
#endif

					if(Cond_Cal_Status)
						MemoryWrite(Test_page, OFFSET_TEST_COND, 4, (uint8_t *) &Conductivity);

					if(Conductivity > 62)
						IS = 0.000016 * Conductivity;
					else
						IS = 0.00001 * Conductivity;

#ifdef TESTING_MODE
					if(g_QCSolution >= 1 && g_QCSolution <= 5)
					{
						if(g_QCSolution == 1)
							IS = .0456;
						else if(g_QCSolution == 2)
							IS = .0437;
						else if(g_QCSolution == 3)
							IS = .0107;
						else if(g_QCSolution == 4)
							IS = .00757;
						else if(g_QCSolution == 5)
							IS = .00615;

						DEBUG_PRINT(UARTprintf("Setting IS to: %d/100000\n");)
					}
#endif

					//				float Ca_Slope_SampT[2];//, Ca_E_Samp_TCor[2];//, Ca_Samp[2];
					float pCa_Rinse;
					// Check if values in the memory are p-values or concentrations
					if(Sols->Ca_EEP_Rinse < 10)	// Values are p-values
					{
						pCa_Rinse = Sols->Ca_EEP_Rinse;
					}
					else	// Values are concentrations
						pCa_Rinse = Calc_pCa(Sols->Ca_EEP_Rinse, T_RS, Sols->IS_RINSE);

					float Ca_M_activity, Ca_M_conc;	// Create variables that need to be used in TH calculation
					for(i = 0; i < ISEs.Ca.size; i++)
					{
						float Ca_Slope_RST = Ca_EEP_Slope[i] * (T_RS + 273.0) / (T_EEP_Cal + 273.0);	// Temperature corrected slope
#ifdef LINEAR_PH_CORR
//						float E_pH = 0;
//						if(Sols->pH_EEP_Cal_2 < 9 && Sols->Ca_EEP_Cal_1 != 0)	// This is Cal 3/Cal 4 setup
//						{
//							float mV_Cal_1 = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[T_Chosen_Ca + ISEs.Ca.index]), OFFSET_CR_CAL_1_MV + ((ISEs.Ca.index + T_Chosen_Ca) * 4), 4));
//							float mV_Rinse = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[T_Chosen_Ca + ISEs.Ca.index]), OFFSET_CR_ISE_1_RINSE + ((ISEs.Ca.index + T_Chosen_Ca) * 4), 4));
//							float T_Cal = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[T_Chosen_Ca + ISEs.Ca.index]), OFFSET_T_CAL, 4));
//							E_pH = (mV_Cal_1 - mV_Rinse) / (Calc_pH_TCor(Sols->pH_EEP_Cal_1, T_Cal, 25, 0, Sols->K_T_pH_Cal_1) - Calc_pH_TCor(Sols->pH_EEP_Rinse, T_Cal, 25, 0, Sols->K_T_pH_Rinse)) * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);
//						}
						float Ca_mpH = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.Ca.index]), OFFSET_CA_1_LOG_K + (i * 4), 4));
						if(Ca_mpH != Ca_mpH)
							Ca_mpH = -1;
//						float E_pH = Ca_mpH * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);
						float E_pH;
						if(pH_Cr_Samp[T_Chosen_pH] < 8)
							E_pH = Ca_mpH * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);
						else
							E_pH = Ca_mpH * (8 - pH_TCor_Rinse);

						// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
						float Ca_Samp = pCa_Rinse + ((Ca_E_Samp[i] - Ca_E_Rinse[i] - E_pH) / Ca_Slope_RST); //pCa
						Ca_M_activity = pow(10, -Ca_Samp);
#elif defined PH_LOG_K
						float log_K_Ca_pH = Build_float(MemoryRead(Cal_page, OFFSET_CA_1_LOG_K + (4 * i), 4));
						if(log_K_Ca_pH == log_K_Ca_pH)
						{
							float E_pH_R = (2 * Ca_Slope_RST * log10((sqrt(pow(10, -pCa_Rinse)) + pow(10, log_K_Ca_pH) * sqrt(pow(10, -pH_TCor_Rinse))) / sqrt(pow(10, -pCa_Rinse))));

							// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
							float Ca_Samp = pCa_Rinse + ((Ca_E_Samp[i] - (Ca_E_Rinse[i] + E_pH_R)) / Ca_Slope_RST); //pCa
							Ca_M_activity = pow(10, -Ca_Samp);

							uint8_t j;
							for(j = 0; j < 10; j++)
							{
								float E_pH_S = (2 * Ca_Slope_RST * log10((sqrt(Ca_M_activity) + pow(10, log_K_Ca_pH) * sqrt(pow(10, -pH_Cr_Samp[T_Chosen_pH]))) / sqrt(Ca_M_activity)));
								Ca_Samp = pCa_Rinse + ((Ca_E_Samp[i] + E_pH_S - (Ca_E_Rinse[i] + E_pH_R)) / Ca_Slope_RST); //pCa
								Ca_M_activity = pow(10, -Ca_Samp);
							}

							//				float Ca_pH_activity = pow(10, -Ca_Samp) + ((1.0 - pow(10, log_K_Ca_pH)) * pow(10, -pH_Cr_Samp[T_Chosen_pH]));
							//				Ca_M_activity = Ca_pH_activity - pow(10, -pH_Cr_Samp[T_Chosen_pH]);
						}
						else
						{
							// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
							float Ca_Samp = pCa_Rinse + ((Ca_E_Samp[i] - Ca_E_Rinse[i]) / Ca_Slope_RST); //pCa
							Ca_M_activity = pow(10, -Ca_Samp);
						}
#else
						// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
						float Ca_Samp = pCa_Rinse + ((Ca_E_Samp[i] - Ca_E_Rinse[i]) / Ca_Slope_RST); //pCa
						Ca_M_activity = pow(10, -Ca_Samp);
#endif	// PH_LOG_K

						Ca_M_conc = Ca_M_activity / Lambda_Ca(T_RS, IS);
						//					float Ca_ppm = Ca_M_conc * 40078;	// [Ca2+]ppm Ca
						Ca_Hardness[i] = Ca_M_conc * 100086.9;	// [Ca Hardness] ppm CaCO3
					}

#ifndef UNIVERSAL_PICKING_FUNCTION
					T_Chosen_Ca = Choose_Ca_Sensor(Cal_Number, Ca_Hardness, Ca_E_Rinse, ISEs, Sols);
#else	// UNIVERSAL_PICKING_FUNCTION
					T_Chosen_Ca = Choose_Sensor(Cal_Number, Ca_Hardness, Ca_E_Rinse, T_Rinse, ISEs.Ca, Sols);
#endif	// UNIVERSAL_PICKING_FUNCTION

#ifndef TESTING_MODE
					if(Ca_Hardness[T_Chosen_Ca] < 0)
						Ca_Hardness[T_Chosen_Ca] = 0;
#endif
					if(Cond_Cal_Status && ISE_Cal_Status[ISEs.Ca.index + T_Chosen_Ca] && ISE_Cal_Status[ISEs.pH_Cr.index + T_Chosen_pH] && ISEs.Ca.size > 0)
						MemoryWrite(Test_page, OFFSET_TEST_CAL_HARDNESS, 4, (uint8_t *) &Ca_Hardness[T_Chosen_Ca]);

					// Recalculate Ca_M_activity for the chosen sensor for use in the TH calculations
					if(1)	// Merely creating a small scope for some of the calculation variables
					{
//						float Ca_Slope_RST = Ca_EEP_Slope[T_Chosen_Ca] * (T_RS + 273.0) / (T_EEP_Cal + 273.0);	// Temperature corrected slope
//#ifdef LINEAR_PH_CORR
////						float E_pH = 0;
////						if(Sols->pH_EEP_Cal_2 < 9 && Sols->Ca_EEP_Cal_1 != 0)	// This is Cal 3/Cal 4 setup
////						{
////							float mV_Cal_1 = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[T_Chosen_Ca + ISEs.Ca.index]), OFFSET_CR_CAL_1_MV + ((ISEs.Ca.index + T_Chosen_Ca) * 4), 4));
////							float mV_Rinse = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[T_Chosen_Ca + ISEs.Ca.index]), OFFSET_CR_ISE_1_RINSE + ((ISEs.Ca.index + T_Chosen_Ca) * 4), 4));
////							float T_Cal = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[T_Chosen_Ca + ISEs.Ca.index]), OFFSET_T_CAL, 4));
////							E_pH = (mV_Cal_1 - mV_Rinse) / (Calc_pH_TCor(Sols->pH_EEP_Cal_1, T_Cal, 25, 0, Sols->K_T_pH_Cal_1) - Calc_pH_TCor(Sols->pH_EEP_Rinse, T_Cal, 25, 0, Sols->K_T_pH_Rinse)) * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);
////						}
//						float Ca_mpH = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[T_Chosen_Ca + ISEs.Ca.index]), OFFSET_CA_1_LOG_K + (T_Chosen_Ca * 4), 4));
//						if(Ca_mpH != Ca_mpH)
//							Ca_mpH = -1;
//						float E_pH;
//						if(pH_Cr_Samp[T_Chosen_pH] < 8)
//							E_pH = Ca_mpH * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);
//						else
//							E_pH = Ca_mpH * (8 - pH_TCor_Rinse);
//
//						// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
//						float Ca_Samp = pCa_Rinse + ((Ca_E_Samp[T_Chosen_Ca] - Ca_E_Rinse[T_Chosen_Ca] - E_pH) / Ca_Slope_RST); //pCa
//						Ca_M_activity = pow(10, -Ca_Samp);
//#elif defined PH_LOG_K
//						float log_K_Ca_pH = Build_float(MemoryRead(Cal_page, OFFSET_CA_1_LOG_K + (4 * T_Chosen_Ca), 4));
//						if(log_K_Ca_pH == log_K_Ca_pH)
//						{
//							float E_pH_R = (2 * Ca_Slope_RST * log10((sqrt(pow(10, -pCa_Rinse)) + pow(10, log_K_Ca_pH) * sqrt(pow(10, -pH_TCor_Rinse))) / sqrt(pow(10, -pCa_Rinse))));
//
//							// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
//							float Ca_Samp = pCa_Rinse + ((Ca_E_Samp[T_Chosen_Ca] - (Ca_E_Rinse[T_Chosen_Ca] + E_pH_R)) / Ca_Slope_RST); //pCa
//							Ca_M_activity = pow(10, -Ca_Samp);
//
//							uint8_t j;
//							for(j = 0; j < 10; j++)
//							{
//								float E_pH_S = (2 * Ca_Slope_RST * log10((sqrt(Ca_M_activity) + pow(10, log_K_Ca_pH) * sqrt(pow(10, -pH_Cr_Samp[T_Chosen_pH]))) / sqrt(Ca_M_activity)));
//								Ca_Samp = pCa_Rinse + ((Ca_E_Samp[T_Chosen_Ca] + E_pH_S - (Ca_E_Rinse[T_Chosen_Ca] + E_pH_R)) / Ca_Slope_RST); //pCa
//								Ca_M_activity = pow(10, -Ca_Samp);
//							}
//
//							//				float Ca_pH_activity = pow(10, -Ca_Samp) + ((1.0 - pow(10, log_K_Ca_pH)) * pow(10, -pH_Cr_Samp[T_Chosen_pH]));
//							//				Ca_M_activity = Ca_pH_activity - pow(10, -pH_Cr_Samp[T_Chosen_pH]);
//						}
//						else
//						{
//							// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
//							float Ca_Samp = pCa_Rinse + ((Ca_E_Samp[T_Chosen_Ca] - Ca_E_Rinse[T_Chosen_Ca]) / Ca_Slope_RST); //pCa
//							Ca_M_activity = pow(10, -Ca_Samp);
//						}
//#else
//						float Ca_Samp = pCa_Rinse + ((Ca_E_Samp[T_Chosen_Ca] + Ca_E_Rinse[T_Chosen_Ca]) / Ca_Slope_RST); //pCa
//						Ca_M_activity = pow(10, -Ca_Samp);
//#endif	// PH_LOG_K
//						Ca_M_conc = Ca_M_activity / Lambda_Ca(T_RS, IS);
//						float Ca_ppm = Ca_M_conc * 40078;	// [Ca2+]ppm Ca
//						//					if(Ca_ppm < 0 && CHECK_NEGATIVE)
//						//						Ca_ppm = 0;
//
//#ifndef TESTING_MODE
//						if(Ca_ppm < 0)
//							Ca_ppm = 0;
//#endif

						// Both Ca_M_conc and Ca_M_activity are used in the TH math
						Ca_M_conc = Ca_Hardness[T_Chosen_Ca] / 100086.9;
						Ca_M_activity = Ca_M_conc * Lambda_Ca(T_RS, IS);
						float Ca_ppm = Ca_M_conc * 40078;
#ifndef TESTING_MODE
						if(Ca_ppm < 0)
							Ca_ppm = 0;
#endif
						if(Cond_Cal_Status && ISE_Cal_Status[ISEs.Ca.index + T_Chosen_Ca] && ISE_Cal_Status[ISEs.pH_Cr.index + T_Chosen_pH] && ISEs.Ca.size > 0)
							MemoryWrite(Test_page, OFFSET_TEST_CALCIUM, 4, (uint8_t *) &Ca_ppm);
					}

					//
					// Magnesium and Total Hardness Measurement
					//
					float pTH_Rinse;
					// Check if values in memory are p-values or concentration
					if(Sols->Ca_EEP_Rinse < 10)	// Values are p-values
					{
						float Mg_100Ca_Rinse = -log10(pow(10, -Sols->TH_EEP_Rinse) - pow(10, -Sols->Ca_EEP_Rinse));
						pTH_Rinse = -log10(pow(10, -Mg_100Ca_Rinse) + pow(10, -5) * pow(10, -Sols->Ca_EEP_Rinse));
					}
					else	// Values are concentration
						pTH_Rinse = Calc_pTH(Sols->Ca_EEP_Rinse, Sols->TH_EEP_Rinse, -5, T_RS, Sols->IS_RINSE);

					for(i = 0; i < ISEs.TH.size; i++)
					{
						float TH_Slope_RST = TH_EEP_Slope[i] * (T_RS + 273) / (T_EEP_Cal + 273); // Temperature corrected slope

#ifdef LINEAR_PH_CORR
//						float E_pH = 0;
//						if(Sols->pH_EEP_Cal_2 < 9 && Sols->Ca_EEP_Cal_1 != 0)	// This is Cal 3/Cal 4 setup
//						{
//							float mV_Cal_1 = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_CR_CAL_1_MV + ((ISEs.TH.index + i) * 4), 4));
//							float mV_Rinse = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_CR_ISE_1_RINSE + ((ISEs.TH.index + i) * 4), 4));
//							float T_Cal = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_T_CAL, 4));
//							E_pH = (mV_Cal_1 - mV_Rinse) / (Calc_pH_TCor(Sols->pH_EEP_Cal_1, T_Cal, 25, 0, Sols->K_T_pH_Cal_1) - Calc_pH_TCor(Sols->pH_EEP_Rinse, T_Cal, 25, 0, Sols->K_T_pH_Rinse)) * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);
//						}
						float TH_mpH = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_MG_1_PH_SLOPE + (i * 4), 4));
						float E_pH = TH_mpH * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);

						float Mg_perCa_Samp = pTH_Rinse + ((TH_E_Samp[i] - TH_E_Rinse[i] - E_pH) / TH_Slope_RST);
						float Mg_perCa = pow(10, -Mg_perCa_Samp);	// a[Mg+X%Ca]

						float Mg_100perCa, Mg_M_activity;

						Mg_100perCa = Mg_perCa + ((1.0 - pow(10, log_K_Ca_Mg)) * Ca_M_activity);
						Mg_M_activity = Mg_100perCa - Ca_M_activity;

#elif defined PH_LOG_K
						float pTH_Samp = pTH_Rinse + ((TH_E_Samp[i] - TH_E_Rinse[i]) / TH_Slope_RST);
						float Mg_perCa = pow(10, -pTH_Samp);	// a[Mg+X%Ca]

						float log_K_TH_pH = Build_float(MemoryRead(Cal_page, OFFSET_TH_1_LOG_K + (4 * i), 4));
						float Mg_100perCa, Mg_M_activity;
						if(log_K_TH_pH == log_K_TH_pH)
						{
							Mg_100perCa = Mg_perCa + ((1.0 - pow(10, log_K_Ca_Mg)) * Ca_M_activity) + ((1.0 - pow(10, log_K_TH_pH)) * pow(10, -pH_Cr_Samp[T_Chosen_pH]));
							Mg_M_activity = Mg_100perCa - Ca_M_activity - pow(10, -pH_Cr_Samp[T_Chosen_pH]);
						}
						else
						{
							Mg_100perCa = Mg_perCa + ((1.0 - pow(10, log_K_Ca_Mg)) * Ca_M_activity);
							Mg_M_activity = Mg_100perCa - Ca_M_activity;
						}

#else
						float pTH_Samp = pTH_Rinse + ((TH_E_Samp[i] - TH_E_Rinse[i]) / TH_Slope_RST);
						float Mg_perCa = pow(10, -pTH_Samp);	// a[Mg+X%Ca]

						float Mg_100perCa = Mg_perCa + ((1.0 - pow(10, log_K_Ca_Mg)) * Ca_M_activity);
						float Mg_M_activity = Mg_100perCa - Ca_M_activity;
#endif	// PH_LOG_K

						float Mg_M_conc = Mg_M_activity / Lambda_Mg(T_RS, IS);

						if(Mg_100perCa > Ca_M_activity)
							TH_corr[i] = (Ca_M_conc + Mg_M_conc) * 100086.9; // [TH]
						else
							TH_corr[i] = Ca_Hardness[T_Chosen_Ca];
					}

#ifndef UNIVERSAL_PICKING_FUNCTION
					T_Chosen_TH = Choose_TH_Sensor(Cal_Number, TH_corr, TH_E_Rinse, ISEs, Sols);
#else	// UNIVERSAL_PICKING_FUNCTION
					T_Chosen_TH = Choose_Sensor(Cal_Number, TH_corr, TH_E_Rinse, T_Rinse, ISEs.TH, Sols);
#endif	// UNIVERSAL_PICKING_FUNCTION

#ifndef TESTING_MODE
					if(TH_corr[T_Chosen_TH] < 0)
						TH_corr[T_Chosen_TH] = 0;
#endif
					if(Cond_Cal_Status && ISE_Cal_Status[ISEs.Ca.index + T_Chosen_Ca] && ISE_Cal_Status[ISEs.TH.index + T_Chosen_TH] && ISE_Cal_Status[ISEs.pH_Cr.index + T_Chosen_pH] && ISEs.Ca.size > 0 && ISEs.TH.size > 0)
					{
						MemoryWrite(Test_page, OFFSET_TEST_TOTAL_HARDNESS, 4, (uint8_t *) &TH_corr[T_Chosen_TH]);

						float Mg_Hardness = TH_corr[T_Chosen_TH] - Ca_Hardness[T_Chosen_Ca];
						float Mg_ppm = Mg_Hardness * 24305.0 / 100086.9;

						MemoryWrite(Test_page, OFFSET_TEST_MAGNESIUM, 4, (uint8_t *) &Mg_ppm);
						MemoryWrite(Test_page, OFFSET_TEST_MAG_HARDNESS, 4, (uint8_t *) &Mg_Hardness);
						MemoryWrite(Test_page, OFFSET_TEST_LOG_K, 4, (uint8_t *) &log_K_Ca_Mg);
					}

//					if(1)	// Merely creating a small scope for variables not needed later
//					{
//						float TH_Slope_RST = TH_EEP_Slope[T_Chosen_TH] * (T_RS + 273) / (T_EEP_Cal + 273); // Temperature corrected slope
//
//#ifdef LINEAR_PH_CORR
////						float E_pH = 0;
////						if(Sols->pH_EEP_Cal_2 < 9 && Sols->Ca_EEP_Cal_1 != 0)	// This is Cal 3/Cal 4 setup
////						{
////							float mV_Cal_1 = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[T_Chosen_TH + ISEs.TH.index]), OFFSET_CR_CAL_1_MV + ((ISEs.TH.index + T_Chosen_TH) * 4), 4));
////							float mV_Rinse = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[T_Chosen_TH + ISEs.TH.index]), OFFSET_CR_ISE_1_RINSE + ((ISEs.TH.index + T_Chosen_TH) * 4), 4));
////							float T_Cal = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[T_Chosen_TH + ISEs.TH.index]), OFFSET_T_CAL, 4));
////							E_pH = (mV_Cal_1 - mV_Rinse) / (Calc_pH_TCor(Sols->pH_EEP_Cal_1, T_Cal, 25, 0, Sols->K_T_pH_Cal_1) - Calc_pH_TCor(Sols->pH_EEP_Rinse, T_Cal, 25, 0, Sols->K_T_pH_Rinse)) * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);
////						}
//						float TH_mpH = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_MG_1_PH_SLOPE + (i * 4), 4));
//						float E_pH = TH_mpH * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);
//
//						float Mg_perCa_Samp = pTH_Rinse + ((TH_E_Samp[T_Chosen_TH] - TH_E_Rinse[T_Chosen_TH] - E_pH) / TH_Slope_RST);
//						float Mg_perCa = pow(10, -Mg_perCa_Samp);	// a[Mg+X%Ca]
//
//						float Mg_100perCa, Mg_M_activity;
//
//						Mg_100perCa = Mg_perCa + ((1.0 - pow(10, log_K_Ca_Mg)) * Ca_M_activity);
//						Mg_M_activity = Mg_100perCa - Ca_M_activity;
//
//#elif defined PH_LOG_K
//						float pTH_Samp = pTH_Rinse + ((TH_E_Samp[T_Chosen_TH] - TH_E_Rinse[T_Chosen_TH]) / TH_Slope_RST);
//						float Mg_perCa = pow(10, -pTH_Samp);	// a[Mg+X%Ca]
//
//						float log_K_TH_pH = Build_float(MemoryRead(Cal_page, OFFSET_TH_1_LOG_K + (4 * i), 4));
//						float Mg_100perCa, Mg_M_activity;
//						if(log_K_TH_pH == log_K_TH_pH)
//						{
//							Mg_100perCa = Mg_perCa + ((1.0 - pow(10, log_K_Ca_Mg)) * Ca_M_activity) + ((1.0 - pow(10, log_K_TH_pH)) * pow(10, -pH_Cr_Samp[T_Chosen_pH]));
//							Mg_M_activity = Mg_100perCa - Ca_M_activity - pow(10, -pH_Cr_Samp[T_Chosen_pH]);
//						}
//						else
//						{
//							Mg_100perCa = Mg_perCa + ((1.0 - pow(10, log_K_Ca_Mg)) * Ca_M_activity);
//							Mg_M_activity = Mg_100perCa - Ca_M_activity;
//						}
//
//#else
//						float pTH_Samp = pTH_Rinse + ((TH_E_Samp[T_Chosen_TH] - TH_E_Rinse[T_Chosen_TH]) / TH_Slope_RST);
//						float Mg_perCa = pow(10, -pTH_Samp);	// a[Mg+X%Ca]
//
//						float Mg_100perCa = Mg_perCa + ((1.0 - pow(10, log_K_Ca_Mg)) * Ca_M_activity);
//						float Mg_M_activity = Mg_100perCa - Ca_M_activity;
//#endif	// PH_LOG_K
//
//						float Mg_M_conc = Mg_M_activity / Lambda_Mg(T_RS, IS);
//						float Mg_Hardness = TH_corr[T_Chosen_TH] - Ca_Hardness[T_Chosen_Ca]; // [Mg Hardness]
//						float Mg_ppm = Mg_M_conc * 24305.0;	// [Mg2+]
//
//						//					if(Mg_Hardness < 0 && CHECK_NEGATIVE)
//						//						Mg_Hardness = 0;
//						//					if(Mg_ppm < 0 && CHECK_NEGATIVE)
//						//						Mg_ppm = 0;
//
//#ifndef TESTING_MODE
//						if(Mg_ppm < 0)
//							Mg_ppm = 0;
//						if(Mg_Hardness < 0)
//							Mg_Hardness = 0;
//#endif
//						if(Cond_Cal_Status && ISE_Cal_Status[ISEs.Ca.index + T_Chosen_Ca] && ISE_Cal_Status[ISEs.TH.index + T_Chosen_TH] && ISE_Cal_Status[ISEs.pH_Cr.index + T_Chosen_pH] && ISEs.Ca.size > 0 && ISEs.TH.size > 0)
//						{
//							MemoryWrite(Test_page, OFFSET_TEST_MAGNESIUM, 4, (uint8_t *) &Mg_ppm);
//							MemoryWrite(Test_page, OFFSET_TEST_MAG_HARDNESS, 4, (uint8_t *) &Mg_Hardness);
//						}
//					}

#ifdef TH_ITERATED_MATH
#ifdef PH_LOG_K
					// Calculate assuming Mg sensor (Nick's math)
					if(ISEs.TH.size > 0)
					{
//						float TH_iterated[2];
//						float log_K_Ca_Mg_Nick = 0.4;

						float pMg_Rinse = Calc_pMg(Sols->Ca_EEP_Rinse, Sols->TH_EEP_Rinse, T_RS, Sols->IS_RINSE);


						// Calculate the activity of both Mg and Ca in rinse
						float a_Mg_R = pow(10, -pMg_Rinse);
						float a_Ca_R = pow(10, -pCa_Rinse);

						for(i = 0; i < 2; i++)
						{
//							log_K_Ca_Mg_Nick[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_MG_1_LOG_K + (i * 4), 4));
							log_K_Ca_Mg_Nick[i] = Build_float(MemoryRead(Cal_page, OFFSET_MG_1_LOG_K + (i * 4), 4));
							if(log_K_Ca_Mg_Nick[i] != log_K_Ca_Mg_Nick[i])
								log_K_Ca_Mg_Nick[i] = -0.5;
							float log_K_TH_pH = Build_float(MemoryRead(Cal_page, OFFSET_TH_1_LOG_K + (4 * i), 4));
							float Mg_EEP_Slope = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_MG_1_SLOPE + (i * 4), 4));
							float Mg_Slope_RST = Mg_EEP_Slope * (T_RS + 273) / (T_EEP_Cal + 273); // Temperature corrected slope

							float Mg_activity_fit;
							if(log_K_TH_pH == log_K_TH_pH)
							{
								float E_Total_R = (2 * Mg_Slope_RST * log10((sqrt(a_Mg_R) + pow(10, log_K_Ca_Mg_Nick[i]) * sqrt(a_Ca_R) + pow(10, log_K_TH_pH) * sqrt(pow(10, -pH_TCor_Rinse))) / sqrt(a_Mg_R)));

								float pMg_Samp = pMg_Rinse + ((TH_E_Samp[i] - (TH_E_Rinse[i] + E_Total_R)) / Mg_Slope_RST);
								Mg_activity_fit = pow(10, -pMg_Samp);

								uint8_t j;
								for(j = 0; j < 10; j++)
								{
				//					float E_Ca = (2 * Mg_Slope_RST * log10((sqrt(Mg_activity_fit) + pow(10, log_K_Ca_Mg_Nick) * sqrt(Ca_M_activity)) / sqrt(Mg_activity_fit)));
				//					float E_pH = (2 * Mg_Slope_RST * log10((sqrt(Mg_activity_fit) + pow(10, log_K_TH_pH) * sqrt(pH_Cr_Samp[T_Chosen_pH])) / sqrt(Mg_activity_fit)));
									float E_Total = (2 * Mg_Slope_RST * log10((sqrt(Mg_activity_fit) + pow(10, log_K_Ca_Mg_Nick[i]) * sqrt(Ca_M_activity) + pow(10, log_K_TH_pH) * sqrt(pow(10, -pH_Cr_Samp[T_Chosen_pH]))) / sqrt(Mg_activity_fit)));
									pMg_Samp = pMg_Rinse + ((TH_E_Samp[i] + E_Total - (TH_E_Rinse[i] + E_Total_R)) / Mg_Slope_RST);
									Mg_activity_fit = pow(10, -pMg_Samp);
								}
							}
							else
							{
								float E_Ca_R = (2 * Mg_Slope_RST * log10((sqrt(a_Mg_R) + pow(10, log_K_Ca_Mg_Nick[i]) * sqrt(a_Ca_R)) / sqrt(a_Mg_R)));

								float pMg_Samp = pMg_Rinse + ((TH_E_Samp[i] - (TH_E_Rinse[i] + E_Ca_R)) / Mg_Slope_RST);
								Mg_activity_fit = pow(10, -pMg_Samp);

								uint8_t j;
								for(j = 0; j < 10; j++)
								{
									float E_Ca = (2 * Mg_Slope_RST * log10((sqrt(Mg_activity_fit) + pow(10, log_K_Ca_Mg_Nick[i]) * sqrt(Ca_M_activity)) / sqrt(Mg_activity_fit)));
									pMg_Samp = pMg_Rinse + ((TH_E_Samp[i] + E_Ca - (TH_E_Rinse[i] + E_Ca_R)) / Mg_Slope_RST);
									Mg_activity_fit = pow(10, -pMg_Samp);
								}
							}

							TH_iterated[i] = (Ca_M_activity / Lambda_Ca(T_RS, IS) + Mg_activity_fit / Lambda_Mg(T_RS, IS)) * 100086.9;
						}
					}
#else
#ifdef PRINT_UART
					// Calculate by ramping ratio/log K until theoretical signal matches observed signal
					if(ISEs.TH.size > 0)
					{
						for(i = 0; i < 2; i++)
						{
							float TH_Slope_RST = TH_EEP_Slope[i] * (T_RS + 273) / (T_EEP_Cal + 273); // Temperature corrected slope
							float TH_mpH = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_MG_1_PH_SLOPE + (i * 4), 4));
							if(TH_mpH != TH_mpH)
								TH_mpH = -2;
							float E_pH = TH_mpH * (pH_Cr_Samp[T_Chosen_pH] - pH_TCor_Rinse);

							float E_Search = TH_E_Samp[i] - TH_E_Rinse[i] - E_pH;
							float E_Theory = E_Search - 1;
							float step_size = .1;
							float ratio = -step_size;
							float K = 1, Mg_Calc, a_Mg, a_Mg_xCa, pTH;
							while(E_Search > E_Theory && step_size >= .001 && K > 0)
							{
								ratio += step_size;
//								Log_K = log10(-0.7936 * ratio + 0.6671);
//								if(Log_K != Log_K)	// Negative K value
//									Log_K = -5;
								K = -0.7936 * ratio + 0.6671;
								if(K < 0)
									K = 0;

								Mg_Calc = ratio * Ca_Hardness[T_Chosen_Ca];
								a_Mg = Mg_Calc * Lambda_Mg(T_RS, IS) / 100086.9;
								a_Mg_xCa = a_Mg + K * Ca_M_activity;
								pTH = -log10(a_Mg_xCa);
								E_Theory = (pTH - pTH_Rinse) * TH_Slope_RST;

								if(E_Search <= E_Theory && step_size >= .001)
								{
									ratio -= step_size;
									step_size /= 10;
								}
							}

							if(K == 0)	// Ratio was high enough just calling K = 0
							{
								pTH = pTH_Rinse + (E_Search / TH_Slope_RST);
								a_Mg_xCa = pow(10, -pTH);
								a_Mg = a_Mg_xCa - K * Ca_M_activity;
								TH_iterated[i] = (Ca_M_activity / Lambda_Ca(T_RS, IS) + a_Mg / Lambda_Mg(T_RS, IS)) * 100086.9;
							}
							else
								TH_iterated[i] = Mg_Calc + Ca_Hardness[T_Chosen_Ca];
						}


#ifndef UNIVERSAL_PICKING_FUNCTION
						T_Chosen_TH_RR = Choose_TH_Sensor(Cal_Number, TH_iterated, TH_E_Rinse, ISEs, Sols);
#else	// UNIVERSAL_PICKING_FUNCTION
						T_Chosen_TH_RR = Choose_Sensor(Cal_Number, TH_iterated, TH_E_Rinse, T_Rinse, ISEs.TH, Sols);
#endif	// UNIVERSAL_PICKING_FUNCTION

#ifdef REPORT_TH_RATIO_RAMP
						if(Cond_Cal_Status && ISE_Cal_Status[ISEs.Ca.index + T_Chosen_Ca] && ISE_Cal_Status[ISEs.TH.index + T_Chosen_TH_RR] && ISE_Cal_Status[ISEs.pH_Cr.index + T_Chosen_pH] && ISEs.Ca.size > 0 && ISEs.TH.size > 0)
						{
							MemoryWrite(Test_page, OFFSET_TEST_TOTAL_HARDNESS, 4, (uint8_t *) &TH_iterated[T_Chosen_TH_RR]);

							float Mg_Hardness = TH_iterated[T_Chosen_TH_RR] - Ca_Hardness[T_Chosen_Ca];
							float Mg_ppm = Mg_Hardness * 24305.0 / 100086.9;

							MemoryWrite(Test_page, OFFSET_TEST_MAGNESIUM, 4, (uint8_t *) &Mg_ppm);
							MemoryWrite(Test_page, OFFSET_TEST_MAG_HARDNESS, 4, (uint8_t *) &Mg_Hardness);
						}
#endif
					}
#endif	// PRINT_UART

//					// Calculate assuming Mg sensor (Nick's math)
//					if(ISEs.TH.size > 0)
//					{
////						float TH_iterated[2];
////						float log_K_Ca_Mg_Nick = 0.4;
//
//						float pMg_Rinse = Calc_pMg(Sols->Ca_EEP_Rinse, Sols->TH_EEP_Rinse, T_RS, Sols->IS_RINSE);
//
//						// Calculate the activity of both Mg and Ca in rinse
//						float a_Mg_R = pow(10, -pMg_Rinse);
//						float a_Ca_R = pow(10, -pCa_Rinse);
//
//						for(i = 0; i < 2; i++)
//						{
////							log_K_Ca_Mg_Nick[i] = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_MG_1_LOG_K + (i * 4), 4));
//							log_K_Ca_Mg_Nick[i] = Build_float(MemoryRead(Cal_page, OFFSET_MG_1_LOG_K + (i * 4), 4));
//							if(log_K_Ca_Mg_Nick[i] != log_K_Ca_Mg_Nick[i])
//								log_K_Ca_Mg_Nick[i] = -0.2;
//							float Mg_EEP_Slope = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.TH.index]), OFFSET_MG_1_SLOPE + (i * 4), 4));
//							float Mg_Slope_RST = Mg_EEP_Slope * (T_RS + 273) / (T_EEP_Cal + 273); // Temperature corrected slope
//							float E_Ca_R = (2 * Mg_Slope_RST * log10((sqrt(a_Mg_R) + pow(10, log_K_Ca_Mg_Nick[i]) * sqrt(a_Ca_R)) / sqrt(a_Mg_R)));
//
//
//							float pMg_Samp = pMg_Rinse + ((TH_E_Samp[i] - (TH_E_Rinse[i] + E_Ca_R)) / Mg_Slope_RST);
//							float Mg_activity_fit = pow(10, -pMg_Samp);
//
//							uint8_t j;
//							for(j = 0; j < 10; j++)
//							{
//								float E_Ca = (2 * Mg_Slope_RST * log10((sqrt(Mg_activity_fit) + pow(10, log_K_Ca_Mg_Nick[i]) * sqrt(Ca_M_activity)) / sqrt(Mg_activity_fit)));
//								pMg_Samp = pMg_Rinse + ((TH_E_Samp[i] + E_Ca - (TH_E_Rinse[i] + E_Ca_R)) / Mg_Slope_RST);
//								Mg_activity_fit = pow(10, -pMg_Samp);
//							}
//
//							TH_iterated[i] = (Ca_M_activity / Lambda_Ca(T_RS, IS) + Mg_activity_fit / Lambda_Mg(T_RS, IS)) * 100086.9;
//						}
//					}
#endif	// PH_LOG_K
#endif	// TH_ITERATED_MATH

					//
					// NH4 Measurement
					//
					float NH4_Alpha_RS = pow(10, -pH_Cr_Samp_RS) / (pow(10, -pH_Cr_Samp_RS) + pow(10, -(0.09018 + 2729.92/T_RS)));

					float pNH4_Rinse;
					if(Sols->Ca_EEP_Rinse < 10)	// Values are p-values
						pNH4_Rinse = Sols->NH4_EEP_Rinse;
					else	// Values are concentration
						pNH4_Rinse = Calc_pNH4(Sols->NH4_EEP_Rinse, pH_TCor_Rinse, 0, T_RS, Sols->IS_RINSE);

					// TODO: Enter potassium and sodium interference values
					float K_interference = 1.5; // ppm
					float Na_interference = 15; // ppm

					for(i = 0; i < ISEs.NH4.size; i++)
					{
						float NH4_Slope_RST = NH4_EEP_Slope[i] * (T_RS + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
						float NH4_Samp = pNH4_Rinse + ((NH4_E_Samp[i] - NH4_E_Rinse[i]) / NH4_Slope_RST);	// pNH4

						float Activity_NH4_K_Na = pow(10, -NH4_Samp);


						float Activity_K = K_interference / 39098.3 * Lambda_K(T_RS, IS);
						float Activity_Na = Na_interference / 22989.8 * Lambda_Na(T_RS, IS);
//#ifdef PH_LOG_K
//						float log_K_NH4_pH = Build_float(MemoryRead(Cal_page, OFFSET_NH4_1_LOG_K + (4 * i), 4));
//						float Activity_NH4;
//						if(log_K_NH4_pH == log_K_NH4_pH)
//						{
//							float Activity_Total = Activity_NH4_K_Na + ((1.0 - pow(10, LOG_K_K_NH4)) * Activity_K) + ((1.0 - pow(10, LOG_K_NA_NH4)) * Activity_Na) + ((1.0 - pow(10, log_K_NH4_pH)) * pow(10, -pH_Cr_Samp_RS));
//							Activity_NH4 = Activity_Total - Activity_K - Activity_Na - pow(10, -pH_Cr_Samp_RS);
//						}
//						else
//						{
//							float Activity_Total = Activity_NH4_K_Na + ((1 - pow(10, LOG_K_K_NH4)) * Activity_K) + ((1 - pow(10, LOG_K_NA_NH4)) * Activity_Na);
//							Activity_NH4 = Activity_Total - Activity_K - Activity_Na;
//						}
//
//#else
						float Activity_Total = Activity_NH4_K_Na + ((1.0 - pow(10, LOG_K_K_NH4)) * Activity_K) + ((1.0 - pow(10, LOG_K_NA_NH4)) * Activity_Na);
						float Activity_NH4 = Activity_Total - Activity_K - Activity_Na;
//#endif // PH_LOG_K

						NH4_Ammonium = Activity_NH4 / Lambda_NH4(T_RS, IS) * 14000;
						NH4_NH3_N_Free[i] = NH4_Ammonium / NH4_Alpha_RS;

						//					NH4_NH3_N_Free[i] = (pow(10, -NH4_Samp[i]) * 14000.0 / NH4_Lambda) - NH4K_Interference; // Free Ammonia
						//					NH4_NH3_N_Total[i] = NH4_NH3_N_Free[i] / NH4_Alpha; // Total ammonia not including monochloramine
					}


#ifndef UNIVERSAL_PICKING_FUNCTION
					T_Chosen_NH4 = Choose_NH4_Sensor(Cal_Number, NH4_NH3_N_Free, NH4_E_Rinse, ISEs, Sols);
#else	// UNIVERSAL_PICKING_FUNCTION
					T_Chosen_NH4 = Choose_Sensor(Cal_Number, NH4_NH3_N_Free, NH4_E_Rinse, T_Rinse, ISEs.NH4, Sols);
#endif	// UNIVERSAL_PICKING_FUNCTION

					if(NH4_NH3_N_Free[T_Chosen_NH4] < 0)
						NH4_NH3_N_Free[T_Chosen_NH4] = 0;

					if(MEASURE_NH4_T1 && pH_Cr_Samp_RS > 8.5)
					{
						DEBUG_PRINT(UARTprintf("Not saving NH4 because of pH!\n");)
					}
					else
					{
						if(Cond_Cal_Status && ISE_Cal_Status[ISEs.NH4.index + T_Chosen_NH4] && ISE_Cal_Status[ISEs.pH_Cr.index + T_Chosen_pH] && ISEs.NH4.size > 0)
						{
							MemoryWrite(Test_page, OFFSET_TEST_TOTAL_NH4, 4, (uint8_t *) &NH4_NH3_N_Free[T_Chosen_NH4]);

							float NH4_Alpha_Therm = pow(10, -pH_Cr_Samp[T_Chosen_pH]) / (pow(10, -pH_Cr_Samp[T_Chosen_pH]) + pow(10, -(0.09018 + 2729.92/T_Therm)));
							NH4_Ammonium = NH4_NH3_N_Free[T_Chosen_NH4] * NH4_Alpha_Therm;

							if(NH4_Ammonium < 0)
								NH4_Ammonium = 0;

							MemoryWrite(Test_page, OFFSET_TEST_FREE_NH4, 4, (uint8_t *) &NH4_Ammonium);
						}
					}

//					//
//					// Conductivity Temperature Correction
//					//
//					// Perform temperature correction here after calculations for ISEs so we are using the conductivity at temperature, not the adjusted conductivity
//					Conductivity /= (1 + COND_TCOMP_SAMP*(T_Samp - 25));
//
//#ifndef TESTING_MODE
//					if(Conductivity < 0)
//						Conductivity = 0;
//#endif
//
//					if(Cond_Cal_Status)
//						MemoryWrite(Test_page, OFFSET_TEST_COND, 4, (uint8_t *) &Conductivity);

					//
					//				ui8TChosen_Sensors = T_Chosen_pH | (T_Chosen_Ca << 2) | (T_Chosen_TH << 3) | (T_Chosen_NH4 << 4);
					ui8TChosen_Sensors =
							(T_Chosen_pH << ISEs.pH_Cr.StorBit) |
							(T_Chosen_TH << ISEs.TH.StorBit) |
							(T_Chosen_NH4 << ISEs.NH4.StorBit) |
							(T_Chosen_Ca << ISEs.Ca.StorBit);

					MemoryWrite(Test_page, OFFSET_CHOSEN_SENSORS, 1, &ui8TChosen_Sensors);

					update_Test(Test_Number);

#ifdef PRINT_UART
					DEBUG_PRINT(UARTprintf("Calculated values:\n");)

					for(i = 0; i < ISEs.pH_H2.size; i++)
						{DEBUG_PRINT(UARTprintf("pH (H2) %d\t", i + 1);)}
					for(i = 0; i < ISEs.pH_Cr.size; i++)
						{DEBUG_PRINT(UARTprintf("pH (Cr) %d\t", i + 1);)}
					for(i = 0; i < ISEs.TH.size; i++)
						{DEBUG_PRINT(UARTprintf("TH %d\t", i + 1);)}
					for(i = 0; i < ISEs.NH4.size; i++)
						{DEBUG_PRINT(UARTprintf("NH4 %d\t", i + 1);)}
					for(i = 0; i < ISEs.Ca.size; i++)
						{DEBUG_PRINT(UARTprintf("Ca %d\t", i + 1);)}
					DEBUG_PRINT(UARTprintf("Conductivity\tTherm Temp\tRS Temp");)
					DEBUG_PRINT(UARTprintf("\n");)

					for(i = 0; i < ISEs.pH_H2.size; i++)
						{DEBUG_PRINT(UARTprintf("=%d/1000\t", (int) (pH_H2_Samp[i] * 1000));)}
					for(i = 0; i < ISEs.pH_Cr.size; i++)
						{DEBUG_PRINT(UARTprintf("=%d/1000\t", (int) (pH_Cr_Samp[i] * 1000));)}
					for(i = 0; i < ISEs.TH.size; i++)
						{DEBUG_PRINT(UARTprintf("=%d/1000\t", (int) (TH_corr[i] * 1000));)}
					for(i = 0; i < ISEs.NH4.size; i++)
						{DEBUG_PRINT(UARTprintf("=%d/1000\t", (int) (NH4_NH3_N_Free[i] * 1000));)}
					for(i = 0; i < ISEs.Ca.size; i++)
						{DEBUG_PRINT(UARTprintf("=%d/1000\t", (int) (Ca_Hardness[i] * 1000));)}
					DEBUG_PRINT(UARTprintf("=%d/1000\t", (int) (Conductivity * 1000));)
					DEBUG_PRINT(UARTprintf("=%d/1000\t", (int) (T_Therm * 1000));)
					DEBUG_PRINT(UARTprintf("=%d/1000", (int) (T_RS * 1000));)

					DEBUG_PRINT(UARTprintf("\n\n");)

					if(ISEs.TH.size > 0)
					{
						DEBUG_PRINT(UARTprintf("TH Comparison:\n");)
						DEBUG_PRINT(UARTprintf("Deric's Math:\t%d\t%d\n", (int) (TH_corr[0] * 1000), (int) (TH_corr[1] * 1000));)
						DEBUG_PRINT(UARTprintf("Nick's Math:\t%d\t%d\n\n", (int) (TH_iterated[0] * 1000), (int) (TH_iterated[1] * 1000));)
					}
#endif	// PRINT_UART
				}
#endif	// STRAIGHT_TO_CL
			}

#ifdef TESTING_MODE
			uint64_t samp_clock = TimerValueGet64(WTIMER0_BASE);
			DEBUG_PRINT(UARTprintf("Time to samp: %d\n", (uint32_t) ((samp_clock - rinse_clock)/SysCtlClockGet()));)
			DEBUG_PRINT(UARTprintf("Total time elapsed: %d s, or %d m %d s\n", (uint32_t) ((samp_clock - start_clock)/SysCtlClockGet()), (uint32_t) ((samp_clock - start_clock)/SysCtlClockGet())/60,(uint32_t) ((samp_clock - start_clock)/SysCtlClockGet())%60);)
#endif

			//
			// Run Alkalinity after sample but before chlorine
			//
			float Alk_Samp[10] = {nanf(""),nanf(""),nanf(""),nanf(""),nanf(""),nanf(""),nanf(""),nanf(""),nanf(""),nanf("")};
			//			float NH4_NH3_N_Free_T1[3] = {-1, -1, -1};
			//			float NH4_NH3_N_Total_T1[3] = {-1, -1, -1};
			PrintTime();
			float Volume_T1_End = 0;	// Save the steps of T1 to reach endpoint to be used if pH is > 8.5 and NH4 needs to be measured
#ifndef STRAIGHT_TO_CL
			if((gui32Error & ABORT_ERRORS) == 0)
				if(MEASURE_ALKALINITY && ISEs.RunAlk)
				{
					update_Status(STATUS_TEST, OPERATION_ALKALINITY);

					uint8_t ui8Times_mixed = 0;
					uint8_t mixing_index = 0;
					float ISE_E_Samp_T1[10] = {0,0,0,0,0,0,0,0,0,0};
					float *pH_H2_E_Samp_T1 = &ISE_E_Samp_T1[ISEs.pH_H2.index];
					float *pH_Cr_E_Samp_T1 = &ISE_E_Samp_T1[ISEs.pH_Cr.index];
//#ifdef PRINT_UART
//					float *NH4_E_Samp_T1 = &ISE_E_Samp_T1[ISEs.NH4.index];
//#endif
					float pH_Samp_T1[20] = {0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};	// Collect 2 pH readings for each sensor, have to make it 20 for max case
					float *pH_H2_Samp_T1 = &pH_Samp_T1[ISEs.pH_H2.index];
					float *pH_Cr_Samp_T1 = &pH_Samp_T1[ISEs.pH_Cr.index];
					float Volume_T1_Endpoint[10] = {0,0,0,0,0,0,0,0,0,0};	// Sensor 1, Sensor 2, Sensor 3

					//					float NH4_E_Samp_T1[3] = {0,0,0};
					float Alk_Slope[10] = {0,0,0,0,0,0,0,0,0,0};
					float T_Samp_T1[2];
					uint8_t method[10] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};	// {Sensor 1, Sensor 2, Sensor 3}
					uint8_t sensor_points[10] = {0,0,0,0,0,0,0,0,0,0};	// Counter for each sensor to determine which mix each sensor is on

					ConnectMemory(1);

					// Calculate the H2 mV offset of Rinse from Cal 6 - Cal 5 line
					float pH_H2_E_Rinse_Adj[2] = {pH_H2_E_Rinse[0], pH_H2_E_Rinse[1]};
//					for(i = 0; i < 2; i++)
//					{
//						float H2_mV_Rinse = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.pH_H2.index]), OFFSET_CR_ISE_1_RINSE + ((i + ISEs.pH_H2.index) * 4), 4));
//						float H2_pH_Cal_T = Calc_pH_TCor(Sols->pH_EEP_Rinse, T_EEP_Cal, 25, 0, Sols->K_T_pH_Rinse);
//						float H2_Cal_Int = Build_float(MemoryRead(Find_Cal_page(Last_cal_passed[i + ISEs.pH_H2.index]), OFFSET_ISE_1_INT + ((i + ISEs.pH_H2.index) * 4), 4));
//						float H2_Rinse_offset = H2_mV_Rinse - (pH_H2_EEP_Slope[i] * H2_pH_Cal_T + H2_Cal_Int);
//						pH_H2_E_Rinse_Adj[i] -= H2_Rinse_offset;
//
//						DEBUG_PRINT(UARTprintf("H2 %d rinse mV offset: %d / 1000\n", i + 1, (int) (H2_Rinse_offset * 1000));)
//					}

					//					float Steps_Sample[2] = {Steps_PreT1 + Steps_PostT1, Steps_PreT1 + Steps_PostT1};	// First mixing, Second mixing
					//					float Steps_Samp_Endpoint[3];	// Sensor 1, Sensor 2, Sensor 3
					float Volume_Sample = PumpVol_PostT1 + ((Steps_PreT1/1000) * PumpVolRev);	// First mixing, Second mixing
//					float Pump_Ratio = 0.61;	// TOD: Adjust pump ratio for alkalinity here
//					float Steps_Samp_Endpoint = Steps_Sample * Pump_Ratio;

					float PumpVol_T1[2];	// First mixing, Second mixing

					// 1/12/2022: Found our Alkalinity is consistently about 20 ppm high, believe this is caused by first 30 steps of acid pumping
					//	not moving solution but building pressure to move solution, subtracting this amount from calculations to get alkalinity
					float Volume_T1_dead = 16.8 * 30.0/610.0; //TOD: Making a guess here that the volume lost will be the same between pumps and not related to steps, guessing its volume lost throughout the chips //PumpVolRev * .3/Pump_Ratio;


					if(Cond_Cal_Status && ISE_Cal_Status[ISEs.Ca.index + T_Chosen_Ca] && ISE_Cal_Status[ISEs.TH.index + T_Chosen_TH] && ISE_Cal_Status[ISEs.pH_Cr.index + T_Chosen_pH] && ISEs.Ca.size > 0 && ISEs.TH.size > 0)
					{
#ifdef REPORT_TH_RATIO_RAMP
						if(TH_iterated[T_Chosen_TH_RR] > Conductivity * 0.2)
							PumpVol_T1[0] = 0.6586 * TH_iterated[T_Chosen_TH_RR] * Volume_Sample / (Sols->HCl_N * 50044) + 2.066;
						else
						{
							DEBUG_PRINT(UARTprintf("\n");)
							PumpVol_T1[0] = PumpVolRev * .500/Pump_Ratio;
						}
#else
						if(TH_corr[T_Chosen_TH] > Conductivity * 0.2)
							PumpVol_T1[0] = 0.6586 * TH_corr[T_Chosen_TH] * Volume_Sample / (Sols->HCl_N * 50044) + 2.066;
						else
						{
							DEBUG_PRINT(UARTprintf("Sample seems soft, picking starting point based on conductivity\n");)
//							PumpVol_T1[0] = PumpVolRev * .500/Pump_Ratio;
							PumpVol_T1[0] = 0.0235 * Conductivity + 0.7969 + PumpVolRev * .050/Pump_Ratio;	// Fit model based on sodium bicarb to get to endpoint + 50 steps
						}
#endif
					}
					else if(Cond_Cal_Status)	// If we do have conductivity but not hardness, base on that
						PumpVol_T1[0] = 0.0235 * Conductivity + 0.7969 + PumpVolRev * .050/Pump_Ratio;	// Fit model based on sodium bicarb to get to endpoint + 50 steps
					else
						PumpVol_T1[0] = PumpVolRev * .500/Pump_Ratio;	// If we don't have TH or conductivity to start with, set half a revolution

#ifdef TESTING_MODE
					if(g_QCSolution >= 1 && g_QCSolution <= 5)
					{
						DEBUG_PRINT(UARTprintf("This is QC solution %d setting pump volumes\n", g_QCSolution);)
						float Target_Alk;
						if(g_QCSolution == 1)
							Target_Alk = 50;
						if(g_QCSolution == 2)
							Target_Alk = 300;
						if(g_QCSolution == 3)
							Target_Alk = 100;
						if(g_QCSolution == 4)
							Target_Alk = 20;
						if(g_QCSolution == 5)
							Target_Alk = 200;
						PumpVol_T1[0] = Target_Alk * Volume_Sample / (50044.0 * Sols->HCl_N) + Volume_T1_dead;	// Alk SM for QC 1 is ~44, set to
					}
#endif	// TESTING_MODE

					// Double check the calculations, if anythings wrong set to a max of 500 steps or min of 50 steps
					if(PumpVol_T1[0] != PumpVol_T1[0])	// Make sure this is a number, not a NAN
						PumpVol_T1[0] = PumpVolRev * .500/Pump_Ratio;
					else if(PumpVol_T1[0] > PumpVolRev * .500/Pump_Ratio)	// Set the upper limit to 500 steps
						PumpVol_T1[0] = PumpVolRev * .500/Pump_Ratio;
					else if(PumpVol_T1[0] < PumpVolRev * .050/Pump_Ratio)	// Set the lower limit to 50 steps
						PumpVol_T1[0] = PumpVolRev * .050/Pump_Ratio;

					PumpVol_T1[1] = PumpVol_T1[0] + PumpVolRev * .050/Pump_Ratio;	// Add 50 steps as initial guess for second mix, this gets recalculated later if pH is below 4.5

#ifdef PRINT_UART
					DEBUG_PRINT(UARTprintf("Subtracting %d nL of T1 from calculations, this is equivalent to 30 steps on 2020 pump heads\n", (int) (Volume_T1_dead * 1000));)

					DEBUG_PRINT(UARTprintf("Volume of T1: %d, %d nL\n", (int) (PumpVol_T1[0] * 1000), (int) (PumpVol_T1[1] * 1000));)
					DEBUG_PRINT(UARTprintf("pH H2 Cal Status:");)
					for(i = 0; i < ISEs.pH_H2.size; i++)
					{
						DEBUG_PRINT(UARTprintf(" %u", ISE_Cal_Status[ISEs.pH_H2.index + i]);)
					}
					DEBUG_PRINT(UARTprintf("\n");)
					DEBUG_PRINT(UARTprintf("pH Cr Cal Status:");)
					for(i = 0; i < ISEs.pH_Cr.size; i++)
					{
						DEBUG_PRINT(UARTprintf(" %u", ISE_Cal_Status[ISEs.pH_Cr.index + i]);)
					}
					DEBUG_PRINT(UARTprintf("\n");)
#endif

					uint8_t pH_passed = 0;
					if(ISEs.pH_H2.size > 0)
						for(i = 0; i < (ISEs.pH_H2.size); i++)
							pH_passed += ISE_Cal_Status[ISEs.pH_H2.index + i];
					else
						for(i = 0; i < (ISEs.pH_Cr.size); i++)
							pH_passed += ISE_Cal_Status[ISEs.pH_Cr.index + i];

					uint8_t Alk_Size = ISEs.pH_H2.size > 0 ? ISEs.pH_H2.size : ISEs.pH_Cr.size;

					// while at least one sensor is calibrated and no endpoint has been found and haven't reached max mixes
					//					while((pH_Cal_Status[0] || pH_Cal_Status[1]) && (Steps_T1_Endpoint[0] == 0 && Steps_T1_Endpoint[1] == 0) && ui8Times_mixed < (MAX_TIMES_TO_MIX + 1) && (gui32Error & ABORT_ERRORS) == 0)

#ifndef PREMIX_ALK
					while((pH_passed > 0) && (FindArrayMax(Volume_T1_Endpoint, Alk_Size) == 0) && (FindArrayMin(Volume_T1_Endpoint, Alk_Size) == 0) && ui8Times_mixed < (MAX_TIMES_TO_MIX + 1) && (gui32Error & ABORT_ERRORS) == 0)
					{
						uint8_t in_range = 0;	// Need two points with pH below 6.3

						uint8_t T1_cond_check = 1;
						uint8_t T1_priming_index = 0;

						//						if((gui32Error & ABORT_ERRORS) == 0)	// Prime everytime before long sample pull because we are pushing bubble back in when puming mixed plug
						while(((gui32Error & ABORT_ERRORS) == 0) && T1_cond_check > 0 && T1_priming_index < 3)
						{
							if(T1_priming_index > 0)	// If the check failed on the first go separate the first plug from the next ones with an air bubble
							{
								RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
								PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 1);
								userDelay(valve_delay_after_air, 1);
							}

							// Prime a little T1 before test to clear out any contamination
							DEBUG_PRINT(UARTprintf("Priming T1... \n");)
							RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);
							PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_Metering, 1);
#ifdef PRIME_BUFFERS_TEST
							if(T1_priming_index == 0 && ui8Times_mixed == 0)	// Only do an extra large prime on the very first priming
							{
								// In order to test if a big prime hurts things set biggest prime for testing purposes
								DEBUG_PRINT(UARTprintf("Adding Max Prime... \n");)
								PumpVolume(FW, 200, Speed_Metering, 1);
							}
#endif
							userDelay(valve_delay, 1);

							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
//							PumpVolume(FW, PumpVol_plug + PumpVol_Solution + PumpVol_Rinse - PumpVol_tube_prime_buffers, Speed_Fast, 1);

							// Spliting the pumping so the plug is moving slowly when going over ISEs
							PumpVolume(FW, PumpVol_plug - PumpVol_tube_prime_buffers, Speed_Fast, 1);
							PumpVolume(FW, PumpVol_Solution + PumpVol_Rinse, Speed_Metering, 1);

							T_Samp_T1[mixing_index] = MeasureTemperature(1);

							float Conductivity_T1;// = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);

#ifndef COND_SOLUTION_STRUCT
							if(Sols->pH_EEP_Cal_2 < 9) // This is Cal 3, not Cal 2
							{
								if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
									Conductivity_T1 = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_1, 0);
								else
									Conductivity_T1 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_1, 0);
							}
							else
							{
								if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
									Conductivity_T1 = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_2, 0);
								else
									Conductivity_T1 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);
							}
#else
							Conductivity_T1 = MeasureConductivity(Sols, 0);
#endif

							Conductivity_T1 = (Conductivity_T1 / (1 + COND_TCOMP_SAMP*(T_Samp_T1[mixing_index] - 25)));

							DEBUG_PRINT(UARTprintf("Temp corrected cond Samp + T1: %d uS/cm * 1000\n", (int) ((Conductivity_T1) * 1000));)

							T1_priming_index++;

							if(Conductivity_T1 > 150 + Conductivity)
							{
								T1_cond_check = 0;
							}
							else
							{
								gui32Error |= T1_PRIME_COND_ERROR; // Update error
								update_Error();
							}
						}

						PumpVolume(FW, PumpVol_sample_rinse - (PumpVol_plug + PumpVol_Solution + PumpVol_Rinse), Speed_Fast, 1);
						FindPossitionZeroPump();
						userDelay(valve_delay, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
						PumpStepperRunStepSpeed_AbortReady(FW, 2000, Speed_Fast);	// Going to leave this as steps to keep volume of air to a miniumum before metering T1

						T1_cond_check = 1;

						//
						// Alkalinity, T1 Mixing
						//
						DEBUG_PRINT(UARTprintf("Mixing %d steps of T1, or %d nL... \n", (int) (PumpVol_T1[mixing_index] * Pump_Ratio * 1000 / PumpVolRev), (int) (PumpVol_T1[mixing_index] * 1000));)

						if(ALK_MIX_IN_AIR)
						{
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
							DEBUG_PRINT(UARTprintf("Pumping large air plug so arrays and reference are uncovered during mixing\n");)
							PumpStepperRunStepSpeed_AbortReady(FW, 7000 + 1000, Speed_Fast);
						}
						userDelay(valve_delay_after_air, 1);

#ifdef ALK_MIX_TWO_PLUGS
						// Pump buffer and solution
						if(ALK_MIX_TWO_PLUGS)
						{
							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
							PumpStepperRunStepSpeed_AbortReady(FW, Steps_PreT1, Speed_Metering);
							userDelay(valve_delay_metering, 1);
//							if(Steps_T1[mixing_index] <= 600)
//							{
								RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
								PumpVolume(FW, PumpVol_T1[mixing_index], Speed_Metering, 1);
								userDelay(valve_delay_metering, 1);
//							}
//							else
//							{
//								uint16_t Steps_to_go = Steps_T1[mixing_index];
//								while(Steps_to_go > 600)
//								{
//									Steps_to_go -= 600;
//									RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
//									PumpStepperRunStepSpeed_AbortReady(FW, 600, Speed_Metering);
//									userDelay(valve_delay_metering, 1);
//									RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
//									PumpStepperRunStepSpeed_AbortReady(FW, 400, Speed_Metering);
//									userDelay(valve_delay_metering, 1);
//								}
//								RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
//								PumpStepperRunStepSpeed_AbortReady(FW, Steps_to_go, Speed_Metering);
//								userDelay(valve_delay_metering, 1);
//							}

							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
							PumpVolume(FW, PumpVol_PostT1, Speed_Metering, 1);
							userDelay(valve_delay_metering, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
							PumpVolume(FW, (2 * PumpVolRev + (PumpVolRev - PumpVol_T1[mixing_index])), Speed_Fast, 1);	// Air bubble size set to return pump to zero position
							userDelay(valve_delay_after_air, 1);
						}
#endif

						// Pump buffer and solution
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
						PumpStepperRunStepSpeed_AbortReady(FW, Steps_PreT1, Speed_Metering);
						userDelay(valve_delay_metering, 1);
//						if(Steps_T1[mixing_index] <= 600)
//						{
							RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
							PumpVolume(FW, PumpVol_T1[mixing_index], Speed_Metering, 1);
							userDelay(valve_delay_metering, 1);
//						}
//						else
//						{
//							uint16_t Steps_to_go = Steps_T1[mixing_index];
//							while(Steps_to_go > 600)
//							{
//								Steps_to_go -= 600;
//								RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
//								PumpStepperRunStepSpeed_AbortReady(FW, 600, Speed_Metering);
//								userDelay(valve_delay_metering, 1);
//								RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
//								PumpStepperRunStepSpeed_AbortReady(FW, 400, Speed_Metering);
//								userDelay(valve_delay_metering, 1);
//							}
//							RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
//							PumpStepperRunStepSpeed_AbortReady(FW, Steps_to_go, Speed_Metering);
//							userDelay(valve_delay_metering, 1);
//						}

						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
						PumpVolume(FW, PumpVol_PostT1, Speed_Metering, 1);
						userDelay(valve_delay_metering, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
						//						PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble, Speed_placing);
						//					userDelay(valve_delay, 1);
						//					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
						PumpVolume(FW, PumpVol_follow_T1 - PumpVol_T1[mixing_index] + PumpVol_tube_bubble, Speed_Fast, 1);
						userDelay(valve_delay_after_air, 1);

						// Replace bubble in tube every mix
						RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
						PumpVolume(BW, PumpVol_tube_bubble, Speed_Metering, 1);
						userDelay(valve_delay_metering, 1);

						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
						PumpStepperMix(FW, Steps_cycles, Speed_Fast, mix_cycles);

						userDelay(diffusion_time, 1);	// Delay

						PumpVolume(FW, PumpVol_center_T1 + PumpVol_T1[mixing_index], Speed_Metering, 1);
						SleepValve();

						//						uint16_t Save_pH = 0;
						//						for(i = 0; i < (ISEs.pH_H2.size + ISEs.pH_Cr.size); i++)
						//							Save_pH |= 1 << (ISEs.pH_H2.index + i);
						//						CollectISEmV(ISE_E_Samp_T1, Save_pH, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);
						CollectISEmV(ISE_E_Samp_T1, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);
#else
						while(ui8Times_mixed < 2)
						{
							uint8_t in_range = 0;	// Need two points with pH below 6.3

							if((gui32Error & ABORT_ERRORS) == 0)	// Prime everytime before long sample pull because we are pushing bubble back in when puming mixed plug
							{
								// Prime a little T1 before test to clear out any contamination
								DEBUG_PRINT(UARTprintf("Priming T1... \n");)
								RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);
								PumpStepperRunStepSpeed_AbortReady(FW, Steps_tube_prime_buffers, Speed_Metering);
								userDelay(valve_delay, 1);
							}

							// Pull large sample plug for both alkalinity mixes, found this helps metering
							//						if(ui8Times_mixed == 0)	// First time mixing pull enough sample to fill until pump
							{
								RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
								PumpStepperRunStepSpeed_AbortReady(FW, Steps_sample_rinse, Speed_priming);
								userDelay(valve_delay, 1);
								RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
								PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble, Speed_priming);
							}

							//
							// Alkalinity, T1 Mixing
							//
							//						DEBUG_PRINT(UARTprintf("Mixing %d steps of T1... \n", Steps_T1[mixing_index]);)
							//						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
							//						FindPossitionZeroPump();
							//						userDelay(valve_delay_after_air, 1);
							//
							//						// Pump buffer and solution
							////						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
							//						if(ui8Times_mixed == 0)
							//						{
							//							DEBUG_PRINT(UARTprintf("For first point pulling premix from B2 port!\n");)
							//							RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);		// Move valve to sample
							//						}
							//						else
							//						{
							//							DEBUG_PRINT(UARTprintf("For second point pulling premix from B1 port!\n");)
							//							RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to sample
							//						}
							//						PumpStepperRunStepSpeed_AbortReady(FW, Steps_PreT1, Speed_Metering);
							//						PumpStepperRunStepSpeed_AbortReady(FW, 220, Speed_Metering);
							//						PumpStepperRunStepSpeed_AbortReady(FW, Steps_PostT1, Speed_Metering);
							//						userDelay(valve_delay_metering, 1);
							//						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
							////						PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble, Speed_placing);
							//						//					userDelay(valve_delay, 1);
							//						//					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
							//						PumpStepperRunStepSpeed_AbortReady(FW, Steps_follow_T1 - 220 + (2 * Steps_tube_bubble), Speed_placing);
							//						userDelay(valve_delay_after_air, 1);
							//
							//						// Replace bubble in tube every mix
							//						if(ui8Times_mixed == 0)
							//						{
							//							RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);		// Move valve to sample
							//						}
							//						else
							//						{
							//							RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to sample
							//						}
							//						PumpStepperRunStepSpeed_AbortReady(BW, Steps_tube_bubble, Speed_Metering);
							//						userDelay(valve_delay_metering, 1);
							//
							//						RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
							//						PumpStepperRunStepSpeed_AbortReady(BW, Steps_tube_bubble, Speed_Metering);
							//						userDelay(valve_delay_metering, 1);
							//
							//						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
							//						PumpStepperMix(FW, Steps_cycles, Speed_mixing, mix_cycles);
							//
							//						userDelay(diffusion_time, 1);	// Delay
							//
							//						PumpStepperRunStepSpeed_AbortReady(FW, Steps_center_T1 + 220, Speed_placing);
							//						SleepValve();

							if(ui8Times_mixed == 0)
							{
								DEBUG_PRINT(UARTprintf("For first point pulling premix from B2 port!\n");)
								//							RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);		// Move valve to sample
							}
							else
							{
								DEBUG_PRINT(UARTprintf("For second point pulling premix from B1 port!\n");)
								//							RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to sample
							}

							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
							FindPossitionZeroPump();

							for (i = 0; i < Number_of_bubbles_samp; i++) // Loop over air/solution cycle 3 times for single solution
							{
								RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
								PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble, Speed_ISE);
								if(i == (Number_of_bubbles_samp - 1))
									PumpStepperRunStepSpeed_AbortReady(FW, Steps_Large_air_bubble_samp, Speed_ISE);
								userDelay(valve_delay_after_air, 1);
								//							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);

								if(ui8Times_mixed == 0)
									RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);		// Move valve to sample
								else
									RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to sample

								if(i == 0 && BUBBLES_IN_TUBE)
									PumpStepperRunStepSpeed_AbortReady(FW, Steps_tube_prime + Steps_Sample_Prime, Speed_ISE);
								PumpStepperRunStepSpeed_AbortReady(FW, runSteps_Solution_samp, Speed_ISE);
								if(i != (Number_of_bubbles_samp - 1))
									userDelay(valve_delay, 1);
							}

							if(MEAUSURE_WHILE_PUMPING || PUMP_THEN_MEASURE)	// If we measure while pumping stay at the sample port so we can keep pumping through
								PumpStepperRunStepSpeed_AbortReady(FW, runSteps_Sample + runSteps_plug_samp, Speed_ISE);
							else
							{
								PumpStepperRunStepSpeed_AbortReady(FW, runSteps_Sample, Speed_ISE);
								userDelay(valve_delay, 1);
								RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
								PumpStepperRunStepSpeed_AbortReady(FW, runSteps_plug_samp, Speed_ISE);
							}

							SleepValve();

							// GND RE for ISEs
							IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 0);		// GND RE
							IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);		// Leave CE floating

							ConnectMemory(0);

							PumpStepperRunTimeSpeed_AbortReady(FW, 25, 6000);
							CollectISEmV(ISE_E_Samp_T1, 0xFFFF, 2, PRINT_ISE_TIME_DATA, &ISEs);

							ConnectMemory(1);
#endif

						T_Samp_T1[mixing_index] = MeasureTemperature(1);

#ifdef PRINT_UART
						float Conductivity_T1;// = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);
#ifndef COND_SOLUTION_STRUCT
						if(Sols->pH_EEP_Cal_2 < 9) // This is Cal 3, not Cal 2
						{
							if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
								Conductivity_T1 = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_1, 0);
							else
								Conductivity_T1 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_1, 0);
						}
						else
						{
							if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
								Conductivity_T1 = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_2, 0);
							else
								Conductivity_T1 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);
						}
#else
						Conductivity_T1 = MeasureConductivity(Sols, 0);
#endif

						DEBUG_PRINT(UARTprintf("Conductivity Samp + T1: %d uS/cm * 1000\n", (int) (Conductivity_T1 * 1000));)
						DEBUG_PRINT(UARTprintf("Temp corrected cond Samp + T1: %d uS/cm * 1000\n", (int) ((Conductivity_T1 / (1 + COND_TCOMP_SAMP*(T_Samp_T1[mixing_index] - 25))) * 1000));)
#endif	// PRINT_UART

						//
						// pH Measurement
						//
						DEBUG_PRINT(UARTprintf("Unadjusted H2 readings:");)
						for(i = 0; i < ISEs.pH_H2.size; i++)
						{
							float pH_H2_Slope_Samp_T1T = pH_H2_EEP_Slope[i] * (T_Samp_T1[mixing_index] + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
							pH_H2_Samp_T1[i + (mixing_index * 10)] = pH_TCor_Rinse + ((pH_H2_E_Samp_T1[i] - pH_H2_E_Rinse[i]) / pH_H2_Slope_Samp_T1T); // pH of sample
							DEBUG_PRINT(UARTprintf("\t%d", (int) (pH_H2_Samp_T1[i + (mixing_index * 10)] * 1000));)
						}
						DEBUG_PRINT(UARTprintf("\n");)

						for(i = 0; i < ISEs.pH_Cr.size; i++)
						{
							float pH_Cr_Slope_Samp_T1T = pH_Cr_EEP_Slope[i] * (T_Samp_T1[mixing_index] + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
							pH_Cr_Samp_T1[i + (mixing_index * 10)] = pH_TCor_Rinse + ((pH_Cr_E_Samp_T1[i] - pH_Cr_E_Rinse[i]) / pH_Cr_Slope_Samp_T1T); // pH of sample
						}

						uint8_t Mix_Chosen_pH_Cr = Choose_Sensor(Cal_Number, pH_Cr_Samp_T1, pH_Cr_E_Rinse, T_Rinse, ISEs.pH_Cr, Sols);

//						// Adjust based on daily calibration difference between Rinse and Cal 6
//						// Adjust based on Cr sensor
//						for(i = 0; i < ISEs.pH_H2.size; i++)
//						{
//							float pH_H2_Slope_Samp_T1T = pH_H2_EEP_Slope[i] * (T_Samp_T1[mixing_index] + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
//							pH_H2_Samp_T1[i + (mixing_index * 10)] = pH_TCor_Rinse + ((pH_H2_E_Samp_T1[i] - pH_H2_E_Rinse_Adj[i]) / pH_H2_Slope_Samp_T1T); // pH of sample
//						}


						// Adjust based on Cr sensor
						for(i = 0; i < ISEs.pH_H2.size; i++)
						{
							float pH_H2_Slope_Samp_T1T = pH_H2_EEP_Slope[i] * (T_Samp_T1[mixing_index] + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
							if(pH_Cr_Samp_T1[Mix_Chosen_pH_Cr] < pH_H2_Samp_T1[i] && mixing_index == 0)
							{
								pH_H2_Samp_T1[i] = pH_Cr_Samp_T1[Mix_Chosen_pH_Cr];
								pH_H2_E_Rinse_Adj[i] = -((pH_H2_Samp_T1[i] - pH_TCor_Rinse) * pH_H2_Slope_Samp_T1T - pH_H2_E_Samp_T1[i]);
								DEBUG_PRINT(UARTprintf("Adjusting H2 %d to align with Cr %d: %d\n", i + 1, Mix_Chosen_pH_Cr + 1, (int) (pH_Cr_Samp_T1[Mix_Chosen_pH_Cr] * 1000));)
							}
							else
							{
								float pH_H2_Slope_Samp_T1T = pH_H2_EEP_Slope[i] * (T_Samp_T1[mixing_index] + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
								pH_H2_Samp_T1[i + (mixing_index * 10)] = pH_TCor_Rinse + ((pH_H2_E_Samp_T1[i] - pH_H2_E_Rinse_Adj[i]) / pH_H2_Slope_Samp_T1T); // pH of sample
							}
						}


#ifdef PRINT_UART
						DEBUG_PRINT(UARTprintf("pH of mixed T1:\n");)
						DEBUG_PRINT(UARTprintf("pH H2:");)
						for(i = 0; i < ISEs.pH_H2.size; i++)
							DEBUG_PRINT(UARTprintf("\t%d", (int) (pH_H2_Samp_T1[i + (mixing_index * 10)] * 1000));)
						DEBUG_PRINT(UARTprintf("\n");)
						DEBUG_PRINT(UARTprintf("pH Cr:");)
						for(i = 0; i < ISEs.pH_Cr.size; i++)
							DEBUG_PRINT(UARTprintf("\t%d", (int) (pH_Cr_Samp_T1[i + (mixing_index * 10)] * 1000));)
						DEBUG_PRINT(UARTprintf("\n");)

//						float NH4_NH3_N_Free_T1[2];
//						if(ISEs.NH4.size > 0)
//						{
//							//
//							// NH4 Measurement
//							//
//							float IS_T1;
//							if(Conductivity_T1 > 62)
//								IS_T1 = 0.000016 * Conductivity_T1;
//							else
//								IS_T1 = 0.00001 * Conductivity_T1;
//
//							float NH4_Alpha_T1 = pow(10, -pH_H2_Samp_T1[T_Chosen_pH + (mixing_index * 10)]) / (pow(10, -pH_H2_Samp_T1[T_Chosen_pH + (mixing_index * 10)]) + pow(10, -(0.09018 + 2729.92/T_RS)));
//
//							float pNH4_Rinse;
//							if(Sols->Ca_EEP_Rinse < 10)	// Values are p-values
//								pNH4_Rinse = Sols->NH4_EEP_Rinse;
//							else	// Values are concentration
//								pNH4_Rinse = Calc_pNH4(Sols->NH4_EEP_Rinse, pH_TCor_Rinse, 0, T_RS, Sols->IS_RINSE);
//
//							// TOD: Enter potassium and sodium interference values
//							float K_interference = 1.5; // ppm
//							float Na_interference = 15; // ppm
//
//							//							float NH4_NH3_N_Free_T1[2];
//							for(i = 0; i < ISEs.NH4.size; i++)
//							{
//								float NH4_Slope_RST = NH4_EEP_Slope[i] * (T_RS + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
//								float NH4_Samp = pNH4_Rinse + ((NH4_E_Samp_T1[i] - NH4_E_Rinse[i]) / NH4_Slope_RST);	// pNH4
//
//								float Activity_NH4_K_Na = pow(10, -NH4_Samp);
//
//								float Activity_K = K_interference / 39098.3 * Lambda_K(T_RS, IS_T1);
//								float Activity_Na = Na_interference / 22989.8 * Lambda_Na(T_RS, IS_T1);
//								float Activity_Total = Activity_NH4_K_Na + ((1 - pow(10, LOG_K_K_NH4)) * Activity_K) + ((1 - pow(10, LOG_K_NA_NH4)) * Activity_Na);
//								float Activity_NH4 = Activity_Total - Activity_K - Activity_Na;
//
//								float NH4_Ammonium_T1 = Activity_NH4 / Lambda_NH4(T_RS, IS_T1) * 14000;
//								NH4_NH3_N_Free_T1[i] = NH4_Ammonium_T1 / NH4_Alpha_T1;
//							}
//						}
//
////						if(ISEs.NH4.size > 0)
////							DEBUG_PRINT(UARTprintf("NH4 of mixed T1:\t=%d/1000\t=%d/1000\n\n", (int) (NH4_NH3_N_Free_T1[0] * 1000), (int) (NH4_NH3_N_Free_T1[1] * 1000));)
//
//						for(i = 0; i < ISEs.NH4.size; i++)
//							{DEBUG_PRINT(UARTprintf("NH4 %d, mixed T1:\t%d\t=%d/1000\n", i + 1, (int) (NH4_NH3_N_Free_T1[i] * 1000));)}
#endif	// PRINT_UART

						//
						// This scheme will mix until at least one sensor has found the endpoint or max mixes are reached
						//
						for(i = 0; i < (ISEs.pH_H2.size); i++)	// Check each sensor
						{
							if(Volume_T1_Endpoint[i + ISEs.pH_H2.index] == 0 && ISE_Cal_Status[i + ISEs.pH_H2.index])	// Check for endpoint if pH sensor passed calibration and hasn't already found endpoint
							{
								// if pH sensor was between 4.1 and 4.9 we found the endpoint
								if(pH_H2_Samp_T1[i + mixing_index * 10] >= 4.5 && pH_H2_Samp_T1[i + mixing_index * 10] <= 4.9)
								{
									DEBUG_PRINT(UARTprintf("Found endpoint on pH H2 sensor %u\n", i + 1);)
									Volume_T1_Endpoint[i + ISEs.pH_H2.index] = PumpVol_T1[mixing_index] - Volume_T1_dead;
									//									Steps_Samp_Endpoint[i] = Steps_Sample[mixing_index] * Pump_Ratio;
									in_range++;
									method[i + ISEs.pH_H2.index] = 2;
									sensor_points[i + ISEs.pH_H2.index] += 2;
								}
								else if(pH_H2_Samp_T1[i + (mixing_index * 10)] < 4.5)
								{
									DEBUG_PRINT(UARTprintf("pH H2 sensor %u read below 4.5!\n", i + 1);)
									in_range++;
									sensor_points[i + ISEs.pH_H2.index]++;
								}
							}
							else if(Volume_T1_Endpoint[i + ISEs.pH_H2.index] != 0 && ISE_Cal_Status[i + ISEs.pH_H2.index])	// If sensor found endpoint on first mixing
							{
								in_range++;

								// Check if second mixing (if ran) was closer to 4.5, if it was use this Steps T1 for endpoint
								if(abs_val(pH_H2_Samp_T1[i + 10] - 4.5) < abs_val(pH_H2_Samp_T1[i] - 4.5))
								{
									Volume_T1_Endpoint[i + ISEs.pH_H2.index] = PumpVol_T1[mixing_index] - Volume_T1_dead;
									//									Steps_Samp_Endpoint[i] = Steps_Sample[mixing_index] * Pump_Ratio;
								}
							}
						}

						for(i = 0; i < (ISEs.pH_Cr.size); i++)	// Check each sensor
						{
							if(Volume_T1_Endpoint[i + ISEs.pH_Cr.index] == 0 && ISE_Cal_Status[i + ISEs.pH_Cr.index])	// Check for endpoint if pH sensor passed calibration and hasn't already found endpoint
							{
								// if pH sensor was between 4.1 and 4.9 we found the endpoint
								if(pH_Cr_Samp_T1[i + ISEs.pH_Cr.index + mixing_index * 10] >= 4.5 && pH_Cr_Samp_T1[i + ISEs.pH_Cr.index + mixing_index * 10] <= 4.9)
								{
									DEBUG_PRINT(UARTprintf("Found endpoint on pH Cr sensor %u\n", i + 1);)
									Volume_T1_Endpoint[i + ISEs.pH_Cr.index] = PumpVol_T1[mixing_index] - Volume_T1_dead;
									//									Steps_Samp_Endpoint[i] = Steps_Sample[mixing_index] * Pump_Ratio;
									if(ISEs.pH_H2.size == 0)
										in_range++;
									method[i + ISEs.pH_Cr.index] = 2;
									sensor_points[i + ISEs.pH_Cr.index] += 2;
								}
								else if(pH_Cr_Samp_T1[i + (mixing_index * 10)] < 4.5)
								{
									DEBUG_PRINT(UARTprintf("pH Cr sensor %u read below 4.5!\n", i + 1);)
									if(ISEs.pH_H2.size == 0)
										in_range++;
									sensor_points[i + ISEs.pH_Cr.index]++;
								}
							}
							else if(Volume_T1_Endpoint[i + ISEs.pH_Cr.index] != 0 && ISE_Cal_Status[i + ISEs.pH_Cr.index])	// If sensor found endpoint on first mixing
							{
								if(ISEs.pH_H2.size == 0)
									in_range++;

								// Check if second mixing (if ran) was closer to 4.5, if it was use this Steps T1 for endpoint
								if(abs_val(pH_Cr_Samp_T1[i + 10] - 4.5) < abs_val(pH_Cr_Samp_T1[i] - 4.5))
								{
									Volume_T1_Endpoint[i + ISEs.pH_Cr.index] = PumpVol_T1[mixing_index] - Volume_T1_dead;
									//									Steps_Samp_Endpoint[i] = Steps_Sample[mixing_index] * Pump_Ratio;
								}
							}
						}

						float pH_offset[2] = {0,0};
						if(in_range > 0)	// Check that at least one sensor was in range before incrementing the mixing index
						{
							if(mixing_index == 1) // This is the second mix, want to make sure a sensor has 2 valid points before incrementing the counter
							{
								uint8_t Complete = 0;

								// Check if a sensor has two valid points, if one sensor gets a valid point then the other sensor gets a valid point but neither have both don't increment mixing index
								if(ISEs.pH_H2.size > 0)
								{
									for(i = 0; i < ISEs.pH_H2.size; i++)
									{
										if(sensor_points[i + ISEs.pH_H2.index] >= 2)	// This sensor has 2 valid points
										{
											if(pH_H2_Samp_T1[i + ISEs.pH_H2.index] >= 3.0 || pH_H2_Samp_T1[i + ISEs.pH_H2.index + 10] >= 3.0)	// Want to only consider the points valid if at least one of the mixes is above 3, this is because below 3 we may be pushing the linear range of our sensors
											{
												Complete++;
												mixing_index++;
												break;	// Exit for loop, at least one sensor has both points
											}
										}
									}
								}
								else
								{
									for(i = 0; i < ISEs.pH_Cr.size; i++)
									{
										if(sensor_points[i + ISEs.pH_Cr.index] >= 2)	// This sensor has 2 valid points
										{
											if(pH_Cr_Samp_T1[i + ISEs.pH_Cr.index] >= 3.0 || pH_Cr_Samp_T1[i + ISEs.pH_Cr.index + 10] >= 3.0)	// Want to only consider the points valid if at least one of the mixes is above 3, this is because below 3 we may be pushing the linear range of our sensors
											{
												Complete++;
												mixing_index++;
												break;	// Exit for loop, at least one sensor has both points
											}
										}
									}
								}

								if(Complete == 0)	// No sensors have all points necessary, need to calculate a new mix, move back to first mixing index then decide which mix to keep, one will be thrown away...
								{
									DEBUG_PRINT(UARTprintf("Appears no sensor has enough data to calculate alk, picking a sensor and running again!\n");)
									mixing_index = 0;
									uint8_t Mix_Chosen_pH_1, Mix_Chosen_pH_2;

#ifndef UNIVERSAL_PICKING_FUNCTION
									if(ISEs.Config == PH_H2_CART)
									{
										Mix_Chosen_pH_1 = Choose_pH_Sensor_pHDie(Cal_Number, pH_Samp_T1);
										Mix_Chosen_pH_2 = Choose_pH_Sensor_pHDie(Cal_Number, pH_Samp_T1 + 10);
									}
									else
									{
										if(ISEs.pH_H2.size > 0)
										{
											Mix_Chosen_pH_1 = Choose_pH_H2_Sensor(Cal_Number, pH_Samp_T1, pH_H2_E_Rinse, T_Rinse, ISEs, Sols) + ISEs.pH_H2.index;
											Mix_Chosen_pH_2 = Choose_pH_H2_Sensor(Cal_Number, pH_Samp_T1 + 10, pH_H2_E_Rinse, T_Rinse, ISEs, Sols) + ISEs.pH_H2.index;
										}
										else
										{
											Mix_Chosen_pH_1 = Choose_pH_Sensor(Cal_Number, pH_Samp_T1, pH_Cr_E_Rinse, T_Rinse, ISEs, Sols) + ISEs.pH_Cr.index;
											Mix_Chosen_pH_2 = Choose_pH_Sensor(Cal_Number, pH_Samp_T1 + 10, pH_Cr_E_Rinse, T_Rinse, ISEs, Sols) + ISEs.pH_Cr.index;
										}
									}
#else	// UNIVERSAL_PICKING_FUNCTION

									if(ISEs.pH_H2.size > 0)
									{
										Mix_Chosen_pH_1 = Choose_Sensor(Cal_Number, pH_Samp_T1, pH_H2_E_Rinse, T_Rinse, ISEs.pH_H2, Sols) + ISEs.pH_H2.index;
										Mix_Chosen_pH_2 = Choose_Sensor(Cal_Number, pH_Samp_T1 + 10, pH_H2_E_Rinse, T_Rinse, ISEs.pH_H2, Sols) + ISEs.pH_H2.index;
									}
									else
									{
										Mix_Chosen_pH_1 = Choose_Sensor(Cal_Number, pH_Samp_T1, pH_Cr_E_Rinse, T_Rinse, ISEs.pH_Cr, Sols) + ISEs.pH_Cr.index;
										Mix_Chosen_pH_2 = Choose_Sensor(Cal_Number, pH_Samp_T1 + 10, pH_Cr_E_Rinse, T_Rinse, ISEs.pH_Cr, Sols) + ISEs.pH_Cr.index;
									}
#endif	// UNIVERSAL_PICKING_FUNCTION

									// Look at the difference between the two mixes to predict what the pH actually is
									for(i = 0; i < ISEs.pH_H2.size; i++)
									{
										float Alk_Slope = (((Volume_Sample + (float) (PumpVol_T1[1] - Volume_T1_dead)) * pow(10, -pH_H2_Samp_T1[i + 10])) - ((Volume_Sample + (float) (PumpVol_T1[0] - Volume_T1_dead)) * pow(10, -pH_H2_Samp_T1[i]))) / ((float) (PumpVol_T1[1] - Volume_T1_dead) - (float) (PumpVol_T1[0] - Volume_T1_dead));
										if(Alk_Slope < Sols->HCl_N)
										{
											DEBUG_PRINT(UARTprintf("Based on difference it appears actual pH is lower than what H2 %d measured\n", i + 1);)

//											float pH_offset = 0;
											while(Alk_Slope < Sols->HCl_N)
											{
												pH_offset[i] -= 0.1;
												Alk_Slope = (((Volume_Sample + (float) (PumpVol_T1[1] - Volume_T1_dead)) * pow(10, -(pH_H2_Samp_T1[i + 10] + pH_offset[i]))) - ((Volume_Sample + (float) (PumpVol_T1[0] - Volume_T1_dead)) * pow(10, -(pH_H2_Samp_T1[i] + pH_offset[i])))) / ((float) (PumpVol_T1[1] - Volume_T1_dead) - (float) (PumpVol_T1[0] - Volume_T1_dead));
											}
											pH_offset[i] += 0.1;	// Take one step back up to underestimate the change rather than overestimate
											DEBUG_PRINT(UARTprintf("Estimated to be about %d /1000 lower\n", (int) (pH_offset[i] * 1000));)
											float pH_Min = -log10(Sols->HCl_N * (PumpVol_T1[1] - Volume_T1_dead) / (PumpVol_T1[1] - Volume_T1_dead + Volume_Sample));
											if((pH_H2_Samp_T1[i + 10] + pH_offset[i]) < pH_Min && pH_offset[i] < 0)
											{
												pH_offset[i] = pH_Min - pH_H2_Samp_T1[i + 10];
												DEBUG_PRINT(UARTprintf("This seems unrealistic, setting to %d / 1000 lower\n", (int) (pH_offset[i] * 1000));)
											}
										}
									}


									// Need to decide which mix to keep...
									if(Mix_Chosen_pH_1 == Mix_Chosen_pH_2)
									{
										uint8_t mix = 0;
										if(pH_Samp_T1[Mix_Chosen_pH_1 + 10] < 4.5)
											mix = 1;

										if(mix == 1)	// Keep the second mix data, need to shift the appropriate variables
										{
											T_Samp_T1[0] = T_Samp_T1[1];
											PumpVol_T1[0] = PumpVol_T1[1];
											memcpy(pH_Samp_T1, &pH_Samp_T1[10], 40);	// Copy the pH values from the second mix into the spaces for the first mix
										}

										DEBUG_PRINT(UARTprintf("Picked same sensor on both mixes, going with sensor %d and mix %d!\n", Mix_Chosen_pH_1 + 1, mix + 1);)

									}
									else	// Chose different sensors, keeping the second mix
									{
										DEBUG_PRINT(UARTprintf("Picked different sensors on each mix, going with second mix!\n");)

										T_Samp_T1[0] = T_Samp_T1[1];
										PumpVol_T1[0] = PumpVol_T1[1];
										memcpy(pH_Samp_T1, &pH_Samp_T1[10], 40);	// Copy the pH values from the second mix into the spaces for the first mix
									}
								}
							}

							if(mixing_index == 0)	// If this was first mix, and it was inbounds, calculate the steps for the second mix
							{
								uint8_t Mix_Chosen_pH;

#ifndef UNIVERSAL_PICKING_FUNCTION
								if(ISEs.Config == PH_H2_CART)
									Mix_Chosen_pH = Choose_pH_Sensor_pHDie(Cal_Number, pH_Samp_T1);
								else
									if(ISEs.pH_H2.size > 0)
										Mix_Chosen_pH = Choose_pH_H2_Sensor(Cal_Number, pH_Samp_T1, pH_H2_E_Rinse, T_Rinse, ISEs, Sols);
									else
										Mix_Chosen_pH = Choose_pH_Sensor(Cal_Number, pH_Samp_T1, pH_Cr_E_Rinse, T_Rinse, ISEs, Sols);
#else	// UNIVERSAL_PICKING_FUNCTION
								if(ISEs.pH_H2.size > 0)
									Mix_Chosen_pH = Choose_Sensor(Cal_Number, pH_Samp_T1, pH_H2_E_Rinse, T_Rinse, ISEs.pH_H2, Sols);
								else
									Mix_Chosen_pH = Choose_Sensor(Cal_Number, pH_Samp_T1, pH_Cr_E_Rinse, T_Rinse, ISEs.pH_Cr, Sols);
#endif	// UNIVERSAL_PICKING_FUNCTION

								DEBUG_PRINT(UARTprintf("Using pH of %d pH * 1000 to calculate steps for second mix!\n", (int) ((pH_H2_Samp_T1[Mix_Chosen_pH] + pH_offset[Mix_Chosen_pH]) * 1000));)
								float Volume_Temp = (((float) (PumpVol_T1[0] - Volume_T1_dead) * (Sols->HCl_N - pow(10, -(pH_H2_Samp_T1[Mix_Chosen_pH] + pH_offset[Mix_Chosen_pH]))) + Volume_Sample * (pow(10, -3.6) - pow(10, -(pH_H2_Samp_T1[Mix_Chosen_pH] + pH_offset[Mix_Chosen_pH])))) / (Sols->HCl_N - pow(10, -3.6))) + Volume_T1_dead;


								DEBUG_PRINT(UARTprintf("Steps for second mix calculated to be %d steps, or %d nL!\n", (int) (Volume_Temp * Pump_Ratio * 1000.0 / PumpVolRev), (int) (Volume_Temp * 1000.0));)
								if(Volume_Temp > .450 * PumpVolRev / Pump_Ratio)	// Check
								{
									PumpVol_T1[1] = .450 * PumpVolRev / Pump_Ratio;
									DEBUG_PRINT(UARTprintf("Setting steps of T1 for second mix to 450!\n");)
								}
								else if(Volume_Temp < .05 * PumpVolRev / Pump_Ratio)
								{
									PumpVol_T1[1] = .050 * PumpVolRev / Pump_Ratio;
									DEBUG_PRINT(UARTprintf("Setting steps of T1 for second mix to 50!\n");)

//									if(PumpVol_T1[0] == .050 * PumpVolRev / Pump_Ratio)
//									{
//										DEBUG_PRINT(UARTprintf("Already pumped 50 steps, calculate using the 50 step cycle\n");)
//										for(i = 0; i < ISEs.pH_H2.size; i++)
//										{
//											Volume_T1_Endpoint[i] = ((float) (PumpVol_T1[0] - Volume_T1_dead) * (Sols->HCl_N - pow(10, -pH_H2_Samp_T1[i])) + Volume_Sample * (pow(10, -4.5) - pow(10, -pH_H2_Samp_T1[i]))) / (Sols->HCl_N - pow(10, -4.5));
//										}
//									}
								}
								else
									PumpVol_T1[1] = Volume_Temp;


								if((PumpVol_T1[0] - PumpVol_T1[1]) < PumpVolRev * .025/Pump_Ratio)	// Check that there is at least 25 steps difference between the two pump cycles
								{
									DEBUG_PRINT(UARTprintf("Step difference between the first and second mix is less than 25, recalculating to get to 4.0\n");)
									Volume_Temp = (((float) (PumpVol_T1[0] - Volume_T1_dead) * (Sols->HCl_N - pow(10, -(pH_H2_Samp_T1[Mix_Chosen_pH] + pH_offset[Mix_Chosen_pH]))) + Volume_Sample * (pow(10, -4.0) - pow(10, -(pH_H2_Samp_T1[Mix_Chosen_pH] + pH_offset[Mix_Chosen_pH])))) / (Sols->HCl_N - pow(10, -4.0))) + Volume_T1_dead;
									if(Volume_Temp > .450 * PumpVolRev / Pump_Ratio)	// Check
									{
										Volume_Temp = .450 * PumpVolRev / Pump_Ratio;
										DEBUG_PRINT(UARTprintf("Setting steps of T1 for second mix to 450!\n");)
									}
									else if(Volume_Temp < .05 * PumpVolRev / Pump_Ratio)
									{
										Volume_Temp = .050 * PumpVolRev / Pump_Ratio;
										DEBUG_PRINT(UARTprintf("Setting steps of T1 for second mix to 50!\n");)
									}

									if((PumpVol_T1[0] - Volume_Temp) < PumpVolRev * .025/Pump_Ratio)
									{
										if(pH_H2_Samp_T1[Mix_Chosen_pH] > 3)
										{
//											DEBUG_PRINT(UARTprintf("Step difference between the first and second mix still less than 25 steps, adding 25 steps to original mix\n");)
//											PumpVol_T1[1] = PumpVol_T1[0] + (PumpVolRev * .025/Pump_Ratio);

											DEBUG_PRINT(UARTprintf("Step difference between the first and second mix still less than 25 steps, calculating to shift pH 0.3\n");)
											Volume_Temp = (((float) (PumpVol_T1[0] - Volume_T1_dead) * (Sols->HCl_N - pow(10, -(pH_H2_Samp_T1[Mix_Chosen_pH] + pH_offset[Mix_Chosen_pH]))) + Volume_Sample * (pow(10, -(pH_H2_Samp_T1[Mix_Chosen_pH] + pH_offset[Mix_Chosen_pH] - .3)) - pow(10, -(pH_H2_Samp_T1[Mix_Chosen_pH] + pH_offset[Mix_Chosen_pH])))) / (Sols->HCl_N - pow(10, -(pH_H2_Samp_T1[Mix_Chosen_pH] + pH_offset[Mix_Chosen_pH] - .3)))) + Volume_T1_dead;
											if((PumpVol_T1[0] - Volume_Temp) < PumpVolRev * .025/Pump_Ratio)
											{
												DEBUG_PRINT(UARTprintf("Step difference calculated is less than 25 steps, setting to 25 steps plus original mix\n");)
												PumpVol_T1[1] = PumpVol_T1[0] + (PumpVolRev * .025/Pump_Ratio);
											}
											else
											{
												DEBUG_PRINT(UARTprintf("Pump volume calculated to be %d nL or %d steps\n", (int) (Volume_Temp * 1000), (int) (Volume_Temp * Pump_Ratio * 1000.0 / PumpVolRev));)
												PumpVol_T1[1] = Volume_Temp;
											}
										}
										else
										{
											DEBUG_PRINT(UARTprintf("First mix below 3, then calculated that removing 25 steps would be over 4 so just calculate based on the first mix :(\n");)

											Volume_Temp = (((float) (PumpVol_T1[0] - Volume_T1_dead) * (Sols->HCl_N - pow(10, -(pH_H2_Samp_T1[Mix_Chosen_pH] + pH_offset[Mix_Chosen_pH]))) + Volume_Sample * (pow(10, -4.5) - pow(10, -(pH_H2_Samp_T1[Mix_Chosen_pH] + pH_offset[Mix_Chosen_pH])))) / (Sols->HCl_N - pow(10, -4.5))) + Volume_T1_dead;
											PumpVol_T1[1] = Volume_Temp;
											DEBUG_PRINT(UARTprintf("Step difference between the first and second mix still less than 25 steps, assuming %d calculated steps or %d nL is the endpoint!\n", (int) (Volume_Temp * Pump_Ratio * 1000 / PumpVolRev), (int) (Volume_Temp * 1000));)

											for(i = 0; i < (ISEs.pH_H2.size); i++)
											{
												if(Volume_T1_Endpoint[i + ISEs.pH_H2.index] == 0 && sensor_points[i + ISEs.pH_H2.index] != 0)	// Don't use the calculated value if the sensor already found the endpoint
												{
													Volume_T1_Endpoint[i + ISEs.pH_H2.index] = Volume_Temp - Volume_T1_dead;

													//									Steps_Samp_Endpoint[i] = Steps_Sample[mixing_index] * Pump_Ratio;
													in_range++;
													method[i + ISEs.pH_H2.index] = 5;
													sensor_points[i + ISEs.pH_H2.index]++;
												}
											}

											for(i = 0; i < (ISEs.pH_Cr.size); i++)
											{
												if(Volume_T1_Endpoint[i + ISEs.pH_Cr.index] == 0 && sensor_points[i + ISEs.pH_Cr.index] != 0)	// Don't use the calculated value if the sensor already found the endpoint
												{
													Volume_T1_Endpoint[i + ISEs.pH_Cr.index] = Volume_Temp - Volume_T1_dead;

													//									Steps_Samp_Endpoint[i] = Steps_Sample[mixing_index] * Pump_Ratio;
													if(ISEs.pH_H2.size == 0)
														in_range++;
													method[i + ISEs.pH_Cr.index] = 5;
													sensor_points[i + ISEs.pH_Cr.index]++;
												}
											}
										}
									}
									else
									{
										DEBUG_PRINT(UARTprintf("Steps for second mix calculated to be %d steps, or %d nL!\n", (int) (Volume_Temp * Pump_Ratio * 1000.0 / PumpVolRev), (int) (Volume_Temp * 1000.0));)
										PumpVol_T1[1] = Volume_Temp;
									}
								}

								mixing_index++;
							}

						}
						else	// No sensors are in range
						{
							DEBUG_PRINT(UARTprintf("No pH sensors are in bounds, increasing T1 steps and trying again!\n");)

							// Use the Henderson Haselbalch equation to estimate the amount of acid to pump to reach just past the endpoint
							float HCl_Conc = Sols->HCl_N * ((PumpVol_T1[mixing_index] - Volume_T1_dead) / ((PumpVol_T1[mixing_index] - Volume_T1_dead) + Volume_Sample));
							float A_Conc = (-1*(HCl_Conc * pow(10, pH_Cr_Samp_T1[T_Chosen_pH + mixing_index * 10] - 6.37) + HCl_Conc)) / ((pow(10, pH_Cr_Samp_T1[T_Chosen_pH + mixing_index * 10]- 6.37)/pow(10, pH_Cr_Samp_RS - 6.37)) - 1);
							float Vol_Temp = ((Volume_Sample * A_Conc / Sols->HCl_N)/(1 - A_Conc / Sols->HCl_N)) + Volume_T1_dead + .050 * PumpVolRev / Pump_Ratio;	// Calculate the volume that will reach the endpoint, then add in the dead volume as well as 50 steps to go past the endpoint

							// Check if this is the first mix or if we already have a valid point
							if(mixing_index == 0)
							{
								if(PumpVol_T1[0] >= .500 * PumpVolRev / Pump_Ratio)
								{
									DEBUG_PRINT(UARTprintf("Pumped max and didn't reach endpoint, calculating volume for endpoint\n");)
									for(i = 0; i < ISEs.pH_H2.size; i++)	// Iterate through each sensor
										Volume_T1_Endpoint[i + ISEs.pH_H2.index] = Vol_Temp - Volume_T1_dead - .050 * PumpVolRev / Pump_Ratio;
									for(i = 0; i < ISEs.pH_Cr.size; i++)	// Iterate through each sensor
										Volume_T1_Endpoint[i + ISEs.pH_Cr.index] = Vol_Temp - Volume_T1_dead - .050 * PumpVolRev / Pump_Ratio;
								}
								else	// Didn't already pump max, calculate using the Henderson Haselbalch equation
								{

									if(Vol_Temp >= PumpVol_T1[0] + .050 * PumpVolRev / Pump_Ratio)	// Check the new volume is at least 50 steps greater than the first pump
										PumpVol_T1[0] = Vol_Temp;
									else	// If the difference is less than 50 steps, bump it up to 50 steps
										PumpVol_T1[0] += PumpVol_T1[1];

									PumpVol_T1[1] = PumpVol_T1[0] + .050 * PumpVolRev / Pump_Ratio;

									DEBUG_PRINT(UARTprintf("Calculated %d nL or %d steps for next mix!\n", (int) (PumpVol_T1[0] * 1000), (int) (PumpVol_T1[0] * Pump_Ratio * 1000.0 / PumpVolRev));)

									if(PumpVol_T1[0] > .500 * PumpVolRev / Pump_Ratio)	//	Check that we don't exceed pump maximum
										PumpVol_T1[0] = .500 * PumpVolRev / Pump_Ratio;
								}
							}
							else	// This is second mix, split the difference
							{
								if(PumpVol_T1[0] >= .500 * PumpVolRev / Pump_Ratio)
								{
									// If we pumped max on the first mix but didn't get in on the second mix check if there is room for mixing in between
									// Originally put in place because we saw that for high alk samples it couldn't get a second mix in bounds
									// 3/11/2024: Saw softened samples mix max on first mix, then cut back to min on second and not reach, then calculate based on first mix only
									if(Vol_Temp > PumpVol_T1[1] && Vol_Temp < (PumpVol_T1[0] - .050 * PumpVolRev / Pump_Ratio))
									{
										PumpVol_T1[1] = Vol_Temp;
									}
									else
									{
										// First mix was in range, but second wasn't. Check if there is room between the two mixes to
										DEBUG_PRINT(UARTprintf("First pump was max and got the correct range, second mix didn't reach, calculate only off first mix\n");)

										for(i = 0; i < ISEs.pH_H2.size; i++)
											Volume_T1_Endpoint[i + ISEs.pH_H2.index] = ((float) (PumpVol_T1[0] - Volume_T1_dead) * (Sols->HCl_N - pow(10, -pH_H2_Samp_T1[i])) + Volume_Sample * (pow(10, -4.5) - pow(10, -pH_H2_Samp_T1[i]))) / (Sols->HCl_N - pow(10, -4.5));

										for(i = 0; i < ISEs.pH_Cr.size; i++)
											Volume_T1_Endpoint[i + ISEs.pH_Cr.index] = ((float) (PumpVol_T1[0] - Volume_T1_dead) * (Sols->HCl_N - pow(10, -pH_Cr_Samp_T1[i])) + Volume_Sample * (pow(10, -4.5) - pow(10, -pH_Cr_Samp_T1[i]))) / (Sols->HCl_N - pow(10, -4.5));
									}
								}
								else
								{
									if(Vol_Temp > PumpVol_T1[1] && Vol_Temp < (PumpVol_T1[0] - 50))
										PumpVol_T1[1] = Vol_Temp;
									else
										PumpVol_T1[1] = (PumpVol_T1[0] + PumpVol_T1[1]) / 2;
								}

							}
						}

						if(mixing_index >= 2)	// If we have two valid points for any sensor that passed calibration
						{
							//							DEBUG_PRINT(UARTprintf("Steps of sample calculated to: %d, %d\n", (int) Steps_Sample[0] * Pump_Ratio, (int) Steps_Sample[1] * Pump_Ratio);)

							for(i = 0; i < ISEs.pH_H2.size; i++)	// Iterate through each sensor
							{
								if(Volume_T1_Endpoint[i + ISEs.pH_H2.index] == 0 && ISE_Cal_Status[i + ISEs.pH_H2.index] && sensor_points[i + ISEs.pH_H2.index] >= 2)	// Calculate endpoint if pH sensor passed calibration and hasn't already found endpoint
								{
									//									Steps_Samp_Endpoint[i] = (Steps_Sample[0] + Steps_Sample[1]) / 2 * Pump_Ratio;
									// New alkalinity procedure should either find the endpoint or have both points below the endpoint
									DEBUG_PRINT(UARTprintf("Both pH H2 values for sensor %u landed below 4.5\n", i + 1);)
									Alk_Slope[i + ISEs.pH_H2.index] = (((Volume_Sample + (float) (PumpVol_T1[1] - Volume_T1_dead)) * pow(10, -pH_H2_Samp_T1[i + 10])) - ((Volume_Sample + (float) (PumpVol_T1[0] - Volume_T1_dead)) * pow(10, -pH_H2_Samp_T1[i]))) / ((float) (PumpVol_T1[1] - Volume_T1_dead) - (float) (PumpVol_T1[0] - Volume_T1_dead));
									float B = ((Volume_Sample + (PumpVol_T1[0] - Volume_T1_dead)) * pow(10, -pH_H2_Samp_T1[i])) - (Alk_Slope[i + ISEs.pH_H2.index] * (PumpVol_T1[0] - Volume_T1_dead));
									Volume_T1_Endpoint[i + ISEs.pH_H2.index] = -B/Alk_Slope[i + ISEs.pH_H2.index];
									DEBUG_PRINT(UARTprintf("Slope = %d / 1000, should equal %d / 1000 +/- 0.02\n", (int) (Alk_Slope[i + ISEs.pH_H2.index] * 1000), (int) (Sols->HCl_N * 1000));)
									method[i + ISEs.pH_H2.index] = 3;

									// Run check on algorithm, just for information not doing anything with it now
									float Alk_Check = abs_val(((PumpVol_T1[0] - Volume_T1_dead) * (Sols->HCl_N - pow(10, -pH_H2_Samp_T1[i]))) / (Volume_Sample * (pow(10, -pH_H2_Samp_T1[i + 10]) - pow(10, -pH_H2_Samp_T1[i]))));
									DEBUG_PRINT(UARTprintf("Check:\t%d/1000\n\n", (int) (Alk_Check * 1000));)
								}
							}

							for(i = 0; i < ISEs.pH_Cr.size; i++)	// Iterate through each sensor
							{
								if(Volume_T1_Endpoint[i + ISEs.pH_Cr.index] == 0 && ISE_Cal_Status[i + ISEs.pH_Cr.index] && sensor_points[i + ISEs.pH_Cr.index] >= 2)	// Calculate endpoint if pH sensor passed calibration and hasn't already found endpoint
								{
									//									Steps_Samp_Endpoint[i] = (Steps_Sample[0] + Steps_Sample[1]) / 2 * Pump_Ratio;
									// New alkalinity procedure should either find the endpoint or have both points below the endpoint
									DEBUG_PRINT(UARTprintf("Both pH Cr values for sensor %u landed below 4.5\n", i + 1);)
									Alk_Slope[i + ISEs.pH_Cr.index] = ((((float) Volume_Sample + (float) (PumpVol_T1[1] - Volume_T1_dead)) * pow(10, -pH_Cr_Samp_T1[i + 10])) - ((Volume_Sample + (float) (PumpVol_T1[0] - Volume_T1_dead)) * pow(10, -pH_Cr_Samp_T1[i]))) / ((float) (PumpVol_T1[1] - Volume_T1_dead) - (float) (PumpVol_T1[0] - Volume_T1_dead));
									float B = ((Volume_Sample + (float) (PumpVol_T1[0] - Volume_T1_dead)) * pow(10, -pH_Cr_Samp_T1[i])) - (Alk_Slope[i + ISEs.pH_Cr.index] * (float) (PumpVol_T1[0] - Volume_T1_dead));
									Volume_T1_Endpoint[i + ISEs.pH_Cr.index] = -B/Alk_Slope[i + ISEs.pH_Cr.index];
									DEBUG_PRINT(UARTprintf("Slope = %d / 1000, should equal %d / 1000 +/- 0.02\n", (int) (Alk_Slope[i + ISEs.pH_Cr.index] * 1000), (int) (Sols->HCl_N * 1000));)
									method[i + ISEs.pH_Cr.index] = 3;

									// Run check on algorithm, just for information not doing anything with it now
									float Alk_Check = abs_val(((float) (PumpVol_T1[0] - Volume_T1_dead) * (Sols->HCl_N - pow(10, -pH_Cr_Samp_T1[i]))) / (Volume_Sample * (pow(10, -pH_Cr_Samp_T1[i + 10]) - pow(10, -pH_Cr_Samp_T1[i]))));
									DEBUG_PRINT(UARTprintf("Check:\t%d/1000\n\n", (int) (Alk_Check * 1000));)
								}
							}
						}

						ui8Times_mixed++;
					}	// End of while loop to keep mixing for alkalinity

					for(i = 0; i < (ISEs.pH_H2.size + ISEs.pH_Cr.size); i++)
					{
						if(Volume_T1_Endpoint[i] != 0)
						{
							Alk_Samp[i] = 50044.0 * Sols->HCl_N * Volume_T1_Endpoint[i] / (Volume_Sample);
						}
					}

					if((gui32Error & ABORT_ERRORS) == 0)
					{
						if(ISEs.pH_H2.size > 0)
							T_Chosen_Alk = Choose_Alk_Sensor(Cal_Number, Alk_Samp, pH_H2_E_Rinse, T_Rinse, method, Alk_Slope, ISEs, Sols);
						else
							T_Chosen_Alk = Choose_Alk_Sensor(Cal_Number, Alk_Samp, pH_Cr_E_Rinse, T_Rinse, method, Alk_Slope, ISEs, Sols);

#ifdef PRINT_UART
						// Calculate what Alkalinity would have been using only each point
						DEBUG_PRINT(UARTprintf("Alkalinity calculated if only using first mix:\n");)
						DEBUG_PRINT(UARTprintf("pH H2:");)
						for(i = 0; i < ISEs.pH_H2.size; i++)
						{
							float Volume_Temp = ((float) (PumpVol_T1[0] - Volume_T1_dead) * (Sols->HCl_N - pow(10, -pH_H2_Samp_T1[i])) + Volume_Sample * (pow(10, -4.5) - pow(10, -pH_H2_Samp_T1[i]))) / (Sols->HCl_N - pow(10, -4.5));
							float Alk_Samp_Temp = 50044.0 * Sols->HCl_N * Volume_Temp / (Volume_Sample);
							DEBUG_PRINT(UARTprintf("\t%d", (int) (Alk_Samp_Temp * 1000));)
						}
						DEBUG_PRINT(UARTprintf("\n");)

						DEBUG_PRINT(UARTprintf("pH Cr:");)
						for(i = 0; i < ISEs.pH_Cr.size; i++)
						{
							float Volume_Temp = ((float) (PumpVol_T1[0] - Volume_T1_dead) * (Sols->HCl_N - pow(10, -pH_Cr_Samp_T1[i])) + Volume_Sample * (pow(10, -4.5) - pow(10, -pH_Cr_Samp_T1[i]))) / (Sols->HCl_N - pow(10, -4.5));
							float Alk_Samp_Temp = 50044.0 * Sols->HCl_N * Volume_Temp / (Volume_Sample);
							DEBUG_PRINT(UARTprintf("\t%d", (int) (Alk_Samp_Temp * 1000));)
						}
						DEBUG_PRINT(UARTprintf("\n\n");)

						DEBUG_PRINT(UARTprintf("Alkalinity calculated if only using second mix:\n");)
						DEBUG_PRINT(UARTprintf("pH H2:");)
						for(i = 0; i < ISEs.pH_H2.size; i++)
						{
							float Volume_Temp = ((float) (PumpVol_T1[1] - Volume_T1_dead) * (Sols->HCl_N - pow(10, -pH_H2_Samp_T1[i + 10])) + Volume_Sample * (pow(10, -4.5) - pow(10, -pH_H2_Samp_T1[i + 10]))) / (Sols->HCl_N - pow(10, -4.5));
							float Alk_Samp_Temp = 50044.0 * Sols->HCl_N * Volume_Temp / (Volume_Sample);
							DEBUG_PRINT(UARTprintf("\t%d", (int) (Alk_Samp_Temp * 1000));)
						}
						DEBUG_PRINT(UARTprintf("\n");)

						DEBUG_PRINT(UARTprintf("pH Cr:");)
						for(i = 0; i < ISEs.pH_Cr.size; i++)
						{
							float Volume_Temp = ((float) (PumpVol_T1[1] - Volume_T1_dead) * (Sols->HCl_N - pow(10, -pH_Cr_Samp_T1[i + 10])) + Volume_Sample * (pow(10, -4.5) - pow(10, -pH_Cr_Samp_T1[i + 10]))) / (Sols->HCl_N - pow(10, -4.5));
							float Alk_Samp_Temp = 50044.0 * Sols->HCl_N * Volume_Temp / (Volume_Sample);
							DEBUG_PRINT(UARTprintf("\t%d", (int) (Alk_Samp_Temp * 1000));)
						}
						DEBUG_PRINT(UARTprintf("\n\n");)


//						DEBUG_PRINT(UARTprintf("Alkalinity calculated assuming pH read correctly but acid was diluted:\n");)
//						DEBUG_PRINT(UARTprintf("pH H2:");)
//						for(i = 0; i < ISEs.pH_H2.size; i++)
//						{
//							float Alk_Samp_Temp = 50044.0 * Alk_Slope[i + ISEs.pH_H2.index] * Volume_T1_Endpoint[i + ISEs.pH_H2.index] / (Volume_Sample);
//							DEBUG_PRINT(UARTprintf("\t%d", (int) (Alk_Samp_Temp * 1000));)
//						}
//						DEBUG_PRINT(UARTprintf("\n");)
//
//						DEBUG_PRINT(UARTprintf("pH Cr:");)
//						for(i = 0; i < ISEs.pH_Cr.size; i++)
//						{
//							float Alk_Samp_Temp = 50044.0 * Alk_Slope[i + ISEs.pH_Cr.index] * Volume_T1_Endpoint[i + ISEs.pH_Cr.index] / (Volume_Sample);
//							DEBUG_PRINT(UARTprintf("\t%d", (int) (Alk_Samp_Temp * 1000));)
//						}
//						DEBUG_PRINT(UARTprintf("\n\n");)

						//						DEBUG_PRINT(UARTprintf("Steps T1 Endpoint: %d, %d, %d \n", (int) (Steps_T1_Endpoint[0]), (int) (Steps_T1_Endpoint[1]), (int) (Steps_T1_Endpoint[2]));)

						DEBUG_PRINT(UARTprintf("Steps, Volume T1 Endpoint for H2:");)
						for(i = 0; i < ISEs.pH_H2.size; i++)
							DEBUG_PRINT(UARTprintf("\t%d\t%d", (int) (Volume_T1_Endpoint[i + ISEs.pH_H2.index] * 1000 * Pump_Ratio / PumpVolRev), (int) (Volume_T1_Endpoint[i + ISEs.pH_H2.index] * 1000));)
						DEBUG_PRINT(UARTprintf("\n");)

						DEBUG_PRINT(UARTprintf("Steps, Volume T1 Endpoint for Cr:");)
						for(i = 0; i < ISEs.pH_Cr.size; i++)
							DEBUG_PRINT(UARTprintf("\t%d\t%d", (int) (Volume_T1_Endpoint[i + ISEs.pH_Cr.index] * 1000 * Pump_Ratio / PumpVolRev), (int) Volume_T1_Endpoint[i + ISEs.pH_Cr.index]);)
						DEBUG_PRINT(UARTprintf("\n");)

						if(Volume_T1_Endpoint[T_Chosen_Alk] != 0)
							Volume_T1_End = Volume_T1_Endpoint[T_Chosen_Alk] + Volume_T1_dead;	// Steps_T1_End only used for pumping in NH4 measurement if pH>8.5 case, so add Steps_T1_dead back into endpoint

						//						DEBUG_PRINT(UARTprintf("Steps Sample Endpoint: %d, %d, %d \n", (int) (Steps_Samp_Endpoint[0]), (int) (Steps_Samp_Endpoint[1]), (int) (Steps_Samp_Endpoint[2]));)
						DEBUG_PRINT(UARTprintf("Steps, Volume Sample Endpoint:\t%d\t%d\n", (int) (Volume_Sample * 1000 * Pump_Ratio / PumpVolRev), (int) (Volume_Sample * 1000));)
						DEBUG_PRINT(UARTprintf("Temperature of Mix: %d, %d C * 1000\n", (int) (T_Samp_T1[0] * 1000), (int) (T_Samp_T1[1] * 1000));)
						//						DEBUG_PRINT(UARTprintf("Alkalinity: %d, %d, %d / 1000\n", (int) (Alk_Samp[0] * 1000), (int) (Alk_Samp[1] * 1000), (int) (Alk_Samp[2] * 1000));)
						if(ISEs.pH_H2.size > 0)
							DEBUG_PRINT(UARTprintf("Alkalinity with H2 Sensors:");)
						for(i = 0; i < ISEs.pH_H2.size; i++)
							DEBUG_PRINT(UARTprintf("\t%d", (int) (Alk_Samp[i + ISEs.pH_H2.index] * 1000));)
						DEBUG_PRINT(UARTprintf("\n");)

						if(ISEs.pH_Cr.size > 0)
							DEBUG_PRINT(UARTprintf("Alkalinity with Cr Sensors:");)
						for(i = 0; i < ISEs.pH_Cr.size; i++)
							DEBUG_PRINT(UARTprintf("\t%d", (int) (Alk_Samp[i + ISEs.pH_Cr.index] * 1000));)
						DEBUG_PRINT(UARTprintf("\n");)

						DEBUG_PRINT(UARTprintf("Sensor %d chosen if going off chosen pH Sensor\n", (T_Chosen_pH + 1));)
						DEBUG_PRINT(UARTprintf("Sensor %d chosen if going off chosen Alk Sensor\n\n", (T_Chosen_Alk + 1));)
#endif
						ui8TChosen_Sensors |= T_Chosen_Alk << ISEs.pH_H2.StorBit;
						MemoryWrite(Test_page, OFFSET_CHOSEN_SENSORS, 1, &ui8TChosen_Sensors);

						// Store data from alkalinity
						if(1)
						{
							// Save the volumes to the memory in nL so it will fit in a uint16_t variable
							uint16_t Vol_T1[2];
							Vol_T1[0] = PumpVol_T1[0] * 1000;
							Vol_T1[1] = PumpVol_T1[1] * 1000;
							MemoryWrite(Test_page, OFFSET_STEPS_T1_1, 2, (uint8_t *) &Vol_T1[0]);
							MemoryWrite(Test_page, OFFSET_STEPS_T1_2, 2, (uint8_t *) &Vol_T1[1]);
						}


						MemoryWrite(Test_page, OFFSET_TEST_PH_1_T1_1, 4, (uint8_t *) &pH_Samp_T1[0]);
						MemoryWrite(Test_page, OFFSET_TEST_PH_1_T1_2, 4, (uint8_t *) &pH_Samp_T1[10]);
						MemoryWrite(Test_page, OFFSET_TEST_PH_2_T1_1, 4, (uint8_t *) &pH_Samp_T1[1]);
						MemoryWrite(Test_page, OFFSET_TEST_PH_2_T1_2, 4, (uint8_t *) &pH_Samp_T1[11]);
						MemoryWrite(Test_page, OFFSET_TEST_PH_3_T1_1, 4, (uint8_t *) &pH_Samp_T1[2]);
						MemoryWrite(Test_page, OFFSET_TEST_PH_3_T1_2, 4, (uint8_t *) &pH_Samp_T1[12]);

						// Write the rest of the values to the memory
						for(i = 0; i < 7; i++)
						{
							MemoryWrite(Test_page, OFFSET_ISE_4_T1_1 + (i * 4), 4, (uint8_t *) &pH_Samp_T1[3 + i]);
							MemoryWrite(Test_page, OFFSET_ISE_4_T1_2 + (i * 4), 4, (uint8_t *) &pH_Samp_T1[13 + i]);
						}

						MemoryWrite(Test_page, OFFSET_RAW_T_SAMP_T1_1, 4, (uint8_t *) &T_Samp_T1[0]);
						MemoryWrite(Test_page, OFFSET_RAW_T_SAMP_T1_2, 4, (uint8_t *) &T_Samp_T1[1]);

#ifndef TESTING_MODE
						if(Alk_Samp[T_Chosen_Alk] < 0)
							Alk_Samp[T_Chosen_Alk] = 0;
#endif
						if(ISE_Cal_Status[ISEs.pH_H2.index + T_Chosen_Alk] && ISEs.pH_H2.size > 0)
							MemoryWrite(Test_page, OFFSET_TEST_ALKALINITY, 4, (uint8_t *) &Alk_Samp[T_Chosen_Alk]);

						float Alk_Bicarb = (Alk_Samp[T_Chosen_Alk] - 5.0 * pow(10, (pH_Cr_Samp[T_Chosen_pH] - 10)))/(1.0 + 0.94 * pow(10, (pH_Cr_Samp[T_Chosen_pH] - 10)));
						float CO2_Free = 2.0 * Alk_Bicarb * pow(10, 6 - pH_Cr_Samp[T_Chosen_pH]);

						if(ISE_Cal_Status[ISEs.pH_H2.index + T_Chosen_Alk] && ISE_Cal_Status[ISEs.pH_Cr.index + T_Chosen_pH] && ISEs.pH_H2.size > 0 && ISEs.pH_Cr.size > 0)
							MemoryWrite(Test_page, OFFSET_TEST_CO2, 4, (uint8_t *) &CO2_Free);

						update_Test(Test_Number);
					}

					//					if(ui8Times_mixed == (MAX_TIMES_TO_MIX + 1) && ((Steps_T1_Endpoint[0] == 0 && pH_Cal_Status[0] == 1) || (Steps_T1_Endpoint[1] == 0 && pH_Cal_Status[1] == 1)  || (DIE_REV_D && Steps_T1_Endpoint[2] == 0 && pH_Cal_Status[2] == 1)))
					if(ui8Times_mixed > 2)	// Alkalinity error appears if it mixed more than twice
					{
						gui32Error |= ALK_MIX_OUT_OF_RANGE;
						update_Error();
					}

#ifdef PUMP_ALK_ENDPOINT
					if(PUMP_ALK_ENDPOINT)
					{
						//
						// Alkalinity, T1 Mixing
						//
						if((gui32Error & ABORT_ERRORS) == 0)	// Prime everytime before long sample pull because we are pushing bubble back in when puming mixed plug
						{
							// Prime a little T1 before test to clear out any contamination
							DEBUG_PRINT(UARTprintf("Priming T1... \n");)
							RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);
							PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_Metering, 1);
							userDelay(valve_delay, 1);
						}

						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
						PumpVolume(FW, PumpVol_sample_rinse, Speed_Fast, 1);
						userDelay(valve_delay, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
						PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 1);

						//						DEBUG_PRINT(UARTprintf("Cutting endpoint steps to 50%% to test NH4...\n");)
						//						Steps_T1_Endpoint[T_Chosen_Alk] *= .5;

						Volume_T1_Endpoint[T_Chosen_Alk] += Volume_T1_dead;	// Add back in the dead steps because this variable is only used for pumping from here on
						if(Volume_T1_Endpoint[T_Chosen_Alk] < .050 * 1000 * Pump_Ratio / PumpVolRev)
						{
							DEBUG_PRINT(UARTprintf("Setting endpoint steps to 50, steps, volume T1 Endpoint calculated to:\t%d\t%d\n", (int) (Volume_T1_Endpoint[T_Chosen_Alk] * 1000 * Pump_Ratio / PumpVolRev), (int) (Volume_T1_Endpoint[T_Chosen_Alk] * 1000));)
							Volume_T1_Endpoint[T_Chosen_Alk] = .050 * 1000 * Pump_Ratio / PumpVolRev;
						}
						else
						{
							DEBUG_PRINT(UARTprintf("Mixing %d steps, or %d nL of T1, this should be the endpoint and mix to 4.5... \n", (int) (Volume_T1_Endpoint[T_Chosen_Alk] * 1000 * Pump_Ratio / PumpVolRev), (int) (Volume_T1_Endpoint[T_Chosen_Alk] * 1000));)
						}

						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
						FindPossitionZeroPump();
						userDelay(valve_delay_after_air, 1);

						// Pump buffer and solution
						if(ALK_MIX_TWO_PLUGS)
						{
							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
							PumpStepperRunStepSpeed_AbortReady(FW, Steps_PreT1, Speed_Metering);
							userDelay(valve_delay_metering, 1);
//							if(Steps_T1_Endpoint[T_Chosen_Alk] <= 600)
//							{
								RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
								PumpVolume(FW, Volume_T1_Endpoint[T_Chosen_Alk], Speed_Metering, 1);
								userDelay(valve_delay_metering, 1);
//							}
//							else
//							{
//								uint16_t Steps_to_go = Steps_T1_Endpoint[T_Chosen_Alk];
//								while(Steps_to_go > 600)
//								{
//									Steps_to_go -= 600;
//									RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
//									PumpStepperRunStepSpeed_AbortReady(FW, 600, Speed_Metering);
//									userDelay(valve_delay_metering, 1);
//									RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
//									PumpStepperRunStepSpeed_AbortReady(FW, 400, Speed_Metering);
//									userDelay(valve_delay_metering, 1);
//								}
//								RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
//								PumpStepperRunStepSpeed_AbortReady(FW, Steps_to_go, Speed_Metering);
//								userDelay(valve_delay_metering, 1);
//							}

							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
							PumpVolume(FW, PumpVol_PostT1, Speed_Metering, 1);
							userDelay(valve_delay_metering, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
							PumpVolume(FW, (2 * PumpVolRev + (PumpVolRev - Volume_T1_Endpoint[T_Chosen_Alk])), Speed_Fast, 1);	// Air bubble size set to return pump to zero position
							userDelay(valve_delay_after_air, 1);
						}

						// Pump buffer and solution
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
						PumpStepperRunStepSpeed_AbortReady(FW, Steps_PreT1, Speed_Metering);
						userDelay(valve_delay_metering, 1);
//						if(Steps_T1_Endpoint[T_Chosen_Alk] <= 600)
//						{
							RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
							PumpVolume(FW, Volume_T1_Endpoint[T_Chosen_Alk], Speed_Metering, 1);
							userDelay(valve_delay_metering, 1);
//						}
//						else
//						{
//							uint16_t Steps_to_go = Steps_T1_Endpoint[T_Chosen_Alk];
//							while(Steps_to_go > 600)
//							{
//								Steps_to_go -= 600;
//								RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
//								PumpStepperRunStepSpeed_AbortReady(FW, 600, Speed_Metering);
//								userDelay(valve_delay_metering, 1);
//								RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
//								PumpStepperRunStepSpeed_AbortReady(FW, 400, Speed_Metering);
//								userDelay(valve_delay_metering, 1);
//							}
//							RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
//							PumpStepperRunStepSpeed_AbortReady(FW, Steps_to_go, Speed_Metering);
//							userDelay(valve_delay_metering, 1);
//						}

						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
						PumpVolume(FW, PumpVol_PostT1, Speed_Metering, 1);
						userDelay(valve_delay_metering, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
						//						PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble, Speed_placing);
						//					userDelay(valve_delay, 1);
						//					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
						PumpVolume(FW, PumpVol_follow_T1 - Volume_T1_Endpoint[T_Chosen_Alk] + PumpVol_tube_bubble, Speed_Fast, 1);
						userDelay(valve_delay_after_air, 1);

						// Replace bubble in tube every mix
						RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
						PumpVolume(BW, PumpVol_tube_bubble, Speed_Metering, 1);
						userDelay(valve_delay_metering, 1);

						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
						PumpStepperMix(FW, Steps_cycles, Speed_Fast, mix_cycles);

						userDelay(diffusion_time, 1);	// Delay

						PumpVolume(FW, PumpVol_center_T1 + Volume_T1_Endpoint[T_Chosen_Alk], Speed_Fast, 1);
						SleepValve();

						//						uint16_t Save_pH = 0;
						//						for(i = 0; i < (ISEs.pH_H2.size + ISEs.pH_Cr.size); i++)
						//							Save_pH |= 1 << (ISEs.pH_H2.index + i);
						//						CollectISEmV(ISE_E_Samp_T1, Save_pH, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);
						CollectISEmV(ISE_E_Samp_T1, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);

						T_Samp_T1[0] = MeasureTemperature(1);

						float Conductivity_T1 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);

						DEBUG_PRINT(UARTprintf("Conductivity Samp + T1: %d uS/cm * 1000\n", (int) (Conductivity_T1 * 1000));)
						DEBUG_PRINT(UARTprintf("Temp corrected cond Samp + T1: %d uS/cm * 1000\n", (int) ((Conductivity_T1 / (1 + COND_TCOMP_SAMP*(T_Samp_T1[0] - 25))) * 1000));)

						//
						// pH Measurement
						//
						DEBUG_PRINT(UARTprintf("pH of mixed T1:\n");)
						DEBUG_PRINT(UARTprintf("pH H2:");)
						float pH_H2_Samp_T1_End;
						for(i = 0; i < ISEs.pH_H2.size; i++)
						{
							float pH_H2_Slope_Samp_T1T = pH_H2_EEP_Slope[i] * (T_Samp_T1[0] + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
							pH_H2_Samp_T1_End = pH_TCor_Rinse + ((pH_H2_E_Samp_T1[i] - pH_H2_E_Rinse[i]) / pH_H2_Slope_Samp_T1T); // pH of sample
							DEBUG_PRINT(UARTprintf("\t%d", (int) (pH_H2_Samp_T1_End * 1000));)
						}
						DEBUG_PRINT(UARTprintf("\n");)
						DEBUG_PRINT(UARTprintf("pH Cr:");)
						for(i = 0; i < ISEs.pH_Cr.size; i++)
						{
							float pH_Cr_Slope_Samp_T1T = pH_Cr_EEP_Slope[i] * (T_Samp_T1[0] + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
							float pH_Cr_Samp_T1_End = pH_TCor_Rinse + ((pH_Cr_E_Samp_T1[i] - pH_Cr_E_Rinse[i]) / pH_Cr_Slope_Samp_T1T); // pH of sample
							DEBUG_PRINT(UARTprintf("\t%d", (int) (pH_Cr_Samp_T1_End * 1000));)
						}
						DEBUG_PRINT(UARTprintf("\n");)

						float NH4_NH3_N_Free_T1[2];
						if(ISEs.NH4.size > 0)
						{
							//
							// NH4 Measurement
							//
							float IS_T1;
							if(Conductivity_T1 > 62)
								IS_T1 = 0.000016 * Conductivity_T1;
							else
								IS_T1 = 0.00001 * Conductivity_T1;

							float NH4_Alpha_T1 = pow(10, -pH_H2_Samp_T1_End) / (pow(10, -pH_H2_Samp_T1_End) + pow(10, -(0.09018 + 2729.92/T_RS)));

							float pNH4_Rinse;
							if(Sols->Ca_EEP_Rinse < 10)	// Values are p-values
								pNH4_Rinse = Sols->NH4_EEP_Rinse;
							else	// Values are concentration
								pNH4_Rinse = Calc_pNH4(Sols->NH4_EEP_Rinse, pH_TCor_Rinse, 0, T_RS, Sols->IS_RINSE);

							// TOD: Enter potassium and sodium interference values
							float K_interference = 1.5; // ppm
							float Na_interference = 15; // ppm

							//							float NH4_NH3_N_Free_T1[2];
							for(i = 0; i < ISEs.NH4.size; i++)
							{
								float NH4_Slope_RST = NH4_EEP_Slope[i] * (T_RS + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
								float NH4_Samp = pNH4_Rinse + ((NH4_E_Samp_T1[i] - NH4_E_Rinse[i]) / NH4_Slope_RST);	// pNH4

								float Activity_NH4_K_Na = pow(10, -NH4_Samp);

								float Activity_K = K_interference / 39098.3 * Lambda_K(T_RS, IS_T1);
								float Activity_Na = Na_interference / 22989.8 * Lambda_Na(T_RS, IS_T1);
								float Activity_Total = Activity_NH4_K_Na + ((1 - pow(10, LOG_K_K_NH4)) * Activity_K) + ((1 - pow(10, LOG_K_NA_NH4)) * Activity_Na);
								float Activity_NH4 = Activity_Total - Activity_K - Activity_Na;

								float NH4_Ammonium_T1 = Activity_NH4 / Lambda_NH4(T_RS, IS_T1) * 14000;
								NH4_NH3_N_Free_T1[i] = NH4_Ammonium_T1 / NH4_Alpha_T1;
							}

							DEBUG_PRINT(UARTprintf("NH4 of mixed T1:\t=%d/1000\t=%d/1000\n\n", (int) (NH4_NH3_N_Free_T1[0] * 1000), (int) (NH4_NH3_N_Free_T1[1] * 1000));)
							DEBUG_PRINT(UARTprintf("NH4 of mixed T1 accounting for diultion:\t=%d/1000\t=%d/1000\n\n", (int) ((NH4_NH3_N_Free_T1[0] * 1000.0) * (Volume_Sample + Volume_T1_Endpoint[T_Chosen_Alk]) / Volume_Sample), (int) ((NH4_NH3_N_Free_T1[1] * 1000) * (Volume_Sample + Volume_T1_Endpoint[T_Chosen_Alk]) / Volume_Sample));)
						}
					}
#endif

					// Moved bubble being pushed back up to where the mix plug is being pumped
					//					// Push a little bubble back into T1 tube to help prevent contamination
					//					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
					//					PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble + Steps_tube_bubble, Speed_placing);
					//					userDelay(valve_delay_after_air, 1);
					//					RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to sample
					//					PumpStepperRunStepSpeed_AbortReady(BW, Steps_tube_bubble, Speed_placing);
					//					userDelay(valve_delay, 1);
				}	// End of if(MEASURE_ALKALINITY)

#ifdef TESTING_MODE
			uint64_t alk_clock = TimerValueGet64(WTIMER0_BASE);
			DEBUG_PRINT(UARTprintf("Time to alk: %d\n", (uint32_t) ((alk_clock - samp_clock) / SysCtlClockGet()));)
			DEBUG_PRINT(UARTprintf("Total time elapsed: %d s, or %d m %d s\n", (uint32_t) ((alk_clock - start_clock)/SysCtlClockGet()), (uint32_t) ((alk_clock - start_clock)/SysCtlClockGet())/60,(uint32_t) ((alk_clock - start_clock)/SysCtlClockGet())%60);)
#endif

			// Check if we need to mix T1 for NH4 measurement
			if((gui32Error & ABORT_ERRORS) == 0)
			{
				if(MEASURE_NH4_T1 && pH_Cr_Samp_RS > 8 && ISEs.NH4.size > 0)
				{
					float ISE_E_Samp_T1[10] = {0,0,0,0,0,0,0,0,0,0};
					//					float *pH_H2_E_Samp_T1 = &ISE_E_Samp_T1[ISEs.pH_H2.index];
					//					float *pH_Cr_E_Samp_T1 = &ISE_E_Samp_T1[ISEs.pH_Cr.index];
					float pH_Samp_T1[10];

					if(Volume_T1_End == 0)	// If the endpoint wasn't found or alkalinity wasn't tested just guess... TODO: Base this off hardness
					{
						if(ISEs.Config >= DISINFECTION_CART && ISEs.Config <= DISINFECTION_CART_2CR_6NH4_2CR)	// This is a disinfection cartridge
						{
							// Trying HEPES buffer on T1 port, Simon wants 7.5 uL, doubled because that is cut in half
							Volume_T1_End = 15;
						}
						else	// Not a disinfection cartridge (full cartridge most likely)
						{
//							Volume_T1_End = .075 * PumpVolRev / Pump_Ratio;

							// Because Ca_Hardness is almost 1-1 with alkalinity use the Ca_Hardness value to calculate the Volume T1 Endpoint
							Volume_T1_End = (Ca_Hardness[T_Chosen_Ca] * (((Steps_PreT1/1000) * PumpVolRev) + PumpVol_PostT1)) / (50044.0 * Sols->HCl_N);
						}
					}

					DEBUG_PRINT(UARTprintf("Cutting endpoint steps to 50%% to test NH4...\n");)
					Volume_T1_End *= .5;

					if(Volume_T1_End < .050 * PumpVolRev / Pump_Ratio)
					{
						DEBUG_PRINT(UARTprintf("Settings steps to 50, steps T1 set to:\t%d\n", (int) (Volume_T1_End * 1000.0 * Pump_Ratio / PumpVolRev));)
						Volume_T1_End = .050 * PumpVolRev / Pump_Ratio;
					}
					else if(Volume_T1_End > .50 * PumpVolRev / Pump_Ratio)
					{
						DEBUG_PRINT(UARTprintf("Settings steps to 500, steps T1 set to: %d\n", (int) (Volume_T1_End * 1000.0 * Pump_Ratio / PumpVolRev));)
						Volume_T1_End = .50 * PumpVolRev / Pump_Ratio;
					}
					else
					{
						DEBUG_PRINT(UARTprintf("Mixing %d steps, %d nL of T1, goal is to be 6.3-8 after mix... \n", (int) ((Volume_T1_End * 1000.0 * Pump_Ratio) / PumpVolRev), (int) (Volume_T1_End * 1000.0));)
					}

					//
					// T1 Mixing
					//
					uint8_t T1_cond_check = 1;
					uint8_t T1_priming_index = 0;

					while(((gui32Error & ABORT_ERRORS) == 0) && T1_cond_check > 0 && T1_priming_index < 3)
					{
						if(T1_priming_index > 0)	// If the check failed on the first go separate the first plug from the next ones with an air bubble
						{
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
							PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 1);
							userDelay(valve_delay_after_air, 1);
						}

						// Prime a little T1 before test to clear out any contamination
						DEBUG_PRINT(UARTprintf("Priming T1... \n");)
						RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);
						PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_Metering, 1);
#ifdef PRIME_BUFFERS_TEST
						if(T1_priming_index == 0 && ui8Times_mixed == 0)	// Only do an extra large prime on the very first priming
						{
							// In order to test if a big prime hurts things set biggest prime for testing purposes
							DEBUG_PRINT(UARTprintf("Adding Max Prime... \n");)
							PumpVolume(FW, 200, Speed_Metering, 1);
						}
#endif
						userDelay(valve_delay, 1);

						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
//							PumpVolume(FW, PumpVol_plug + PumpVol_Solution + PumpVol_Rinse - PumpVol_tube_prime_buffers, Speed_Fast, 1);

						// Spliting the pumping so the plug is moving slowly when going over ISEs
						PumpVolume(FW, PumpVol_plug - PumpVol_tube_prime_buffers, Speed_Fast, 1);
						PumpVolume(FW, PumpVol_Solution + PumpVol_Rinse, Speed_Metering, 1);

//						T_Samp_T1[mixing_index] = MeasureTemperature(1);

						float Conductivity_T1;// = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);

#ifndef COND_SOLUTION_STRUCT
						if(Sols->pH_EEP_Cal_2 < 9) // This is Cal 3, not Cal 2
						{
							if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
								Conductivity_T1 = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_1, 0);
							else
								Conductivity_T1 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_1, 0);
						}
						else
						{
							if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
								Conductivity_T1 = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_2, 0);
							else
								Conductivity_T1 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);
						}
#else
						Conductivity_T1 = MeasureConductivity(Sols, 0);
#endif

						Conductivity_T1 = (Conductivity_T1 / (1 + COND_TCOMP_SAMP*(T_Samp - 25)));

						DEBUG_PRINT(UARTprintf("Temp corrected cond Samp + T1: %d uS/cm * 1000\n", (int) ((Conductivity_T1) * 1000));)

						T1_priming_index++;

						if(Conductivity_T1 > 150 + Conductivity)
						{
							T1_cond_check = 0;
						}
						else
						{
							gui32Error |= T1_PRIME_COND_ERROR; // Update error
							update_Error();
						}
					}

					PumpVolume(FW, PumpVol_sample_rinse - (PumpVol_plug + PumpVol_Solution + PumpVol_Rinse), Speed_Fast, 1);
					FindPossitionZeroPump();
					userDelay(valve_delay, 1);
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
					PumpStepperRunStepSpeed_AbortReady(FW, 2000, Speed_Fast);	// Going to leave this as steps to keep volume of air to a miniumum before metering T1

					T1_cond_check = 1;

					// T1 Mixing
					if(ALK_MIX_IN_AIR)
					{
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
						DEBUG_PRINT(UARTprintf("Pumping large air plug so arrays and reference are uncovered during mixing\n");)
						PumpStepperRunStepSpeed_AbortReady(FW, 7000 + 1000, Speed_Fast);
					}
					userDelay(valve_delay_after_air, 1);

					//						DEBUG_PRINT(UARTprintf("Cutting endpoint steps to 50%% to test NH4...\n");)
					//						Steps_T1_Endpoint[T_Chosen_Alk] *= .5;

					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
					FindPossitionZeroPump();
					userDelay(valve_delay_after_air, 1);

					// Pump buffer and solution
					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
					PumpStepperRunStepSpeed_AbortReady(FW, Steps_PreT1, Speed_Metering);
					userDelay(valve_delay_metering, 1);
//					if(Steps_T1_End <= 600)
//					{
						RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
						PumpVolume(FW, Volume_T1_End, Speed_Metering, 1);
						userDelay(valve_delay_metering, 1);
//					}
//					else
//					{
//						uint16_t Steps_to_go = Steps_T1_End;
//						while(Steps_to_go > 600)
//						{
//							Steps_to_go -= 600;
//							RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
//							PumpStepperRunStepSpeed_AbortReady(FW, 600, Speed_Metering);
//							userDelay(valve_delay_metering, 1);
//							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
//							PumpStepperRunStepSpeed_AbortReady(FW, 400, Speed_Metering);
//							userDelay(valve_delay_metering, 1);
//						}
//						RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
//						PumpStepperRunStepSpeed_AbortReady(FW, Steps_to_go, Speed_Metering);
//						userDelay(valve_delay_metering, 1);
//					}

					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
					PumpVolume(FW, PumpVol_PostT1, Speed_Metering, 1);
					userDelay(valve_delay_metering, 1);
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
					//						PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble, Speed_placing);
					//					userDelay(valve_delay, 1);
					//					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
					PumpVolume(FW, PumpVol_follow_T1 - Volume_T1_End + PumpVol_tube_bubble, Speed_Fast, 1);
					userDelay(valve_delay_after_air, 1);

					// Replace bubble in tube every mix
					RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
					PumpVolume(BW, PumpVol_tube_bubble, Speed_Metering, 1);
					userDelay(valve_delay_metering, 1);

					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
					PumpStepperMix(FW, Steps_cycles, Speed_Fast, mix_cycles);

					userDelay(diffusion_time, 1);	// Delay

					PumpVolume(FW, PumpVol_center_T1 + Volume_T1_End, Speed_Metering, 1);
					SleepValve();

					CollectISEmV(ISE_E_Samp_T1, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);

					float Conductivity_T1;// = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);
#ifndef COND_SOLUTION_STRUCT
					if(Sols->pH_EEP_Cal_2 < 9) // This is Cal 3, not Cal 2
					{
						if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
							Conductivity_T1 = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_1, 0);
						else
							Conductivity_T1 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_1, 0);
					}
					else
					{
						if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
							Conductivity_T1 = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_2, 0);
						else
							Conductivity_T1 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);
					}
#else
					Conductivity_T1 = MeasureConductivity(Sols, 0);
#endif

					DEBUG_PRINT(UARTprintf("Conductivity read: =%d/1000\n", (int) (Conductivity_T1 * 1000));)
					if(Conductivity_T1 < 50)
					{
						DEBUG_PRINT(UARTprintf("Setting conductivity to 50 for calculations!\n");)
						Conductivity_T1 = 50;
					}

					for(i = 0; i < ISEs.NH4.size; i++)
					{
						ISE_E_Samp[ISEs.NH4.index + i] = ISE_E_Samp_T1[ISEs.NH4.index + i];
					}

					//								// pH Measurement
					for(i = 0; i < ISEs.pH_Cr.size; i++)
					{
						float pH_Cr_Slope_Samp_T1T = ISE_EEP_Slope[ISEs.pH_Cr.index + i] * (T_RS + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
						pH_Samp_T1[ISEs.pH_Cr.index + i] = pH_TCor_Rinse + ((ISE_E_Samp_T1[ISEs.pH_Cr.index + i] - ISE_E_Rinse[ISEs.pH_Cr.index + i]) / pH_Cr_Slope_Samp_T1T); // pH of sample
					}

					ConnectMemory(1);


#ifndef UNIVERSAL_PICKING_FUNCTION
					uint8_t T_Chosen_pH_T1 = Choose_pH_Sensor(Cal_Number, pH_Samp_T1, &ISE_E_Rinse[ISEs.pH_Cr.index], T_RS, ISEs, Sols);
#else	// UNIVERSAL_PICKING_FUNCTION
					uint8_t T_Chosen_pH_T1 = Choose_Sensor(Cal_Number, pH_Samp_T1, pH_Cr_E_Rinse, T_Rinse, ISEs.pH_Cr, Sols);
#endif	// UNIVERSAL_PICKING_FUNCTION

					//
					// NH4 Measurement
					//
					float IS_T1;
					if(Conductivity_T1 > 62)
						IS_T1 = 0.000016 * Conductivity_T1;
					else
						IS_T1 = 0.00001 * Conductivity_T1;

					float NH4_Alpha_T1 = pow(10, -pH_Samp_T1[T_Chosen_pH_T1]) / (pow(10, -pH_Samp_T1[T_Chosen_pH_T1]) + pow(10, -(0.09018 + 2729.92/T_RS)));

					float pNH4_Rinse;
					if(Sols->Ca_EEP_Rinse < 10)	// Values are p-values
						pNH4_Rinse = Sols->NH4_EEP_Rinse;
					else	// Values are concentration
						pNH4_Rinse = Calc_pNH4(Sols->NH4_EEP_Rinse, pH_TCor_Rinse, 0, T_RS, Sols->IS_RINSE);

					// TODO: Enter potassium and sodium interference values
					float K_interference = 1.5; // ppm
					float Na_interference = 15; // ppm

					for(i = 0; i < ISEs.NH4.size; i++)
					{
						float NH4_Slope_RST = NH4_EEP_Slope[i] * (T_RS + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
						float NH4_Samp = pNH4_Rinse + ((NH4_E_Samp[i] - NH4_E_Rinse[i]) / NH4_Slope_RST);	// pNH4

						float Activity_NH4_K_Na = pow(10, -NH4_Samp);

						float Activity_K = K_interference / 39098.3 * Lambda_K(T_RS, IS_T1);
						float Activity_Na = Na_interference / 22989.8 * Lambda_Na(T_RS, IS_T1);
						float Activity_Total = Activity_NH4_K_Na + ((1 - pow(10, LOG_K_K_NH4)) * Activity_K) + ((1 - pow(10, LOG_K_NA_NH4)) * Activity_Na);
						float Activity_NH4 = Activity_Total - Activity_K - Activity_Na;

						NH4_Ammonium = Activity_NH4 / Lambda_NH4(T_RS, IS_T1) * 14000.0 * (((float) (((Steps_PreT1/1000) * PumpVolRev) + PumpVol_PostT1)) + Volume_T1_End) / ((float) (((Steps_PreT1/1000) * PumpVolRev) + PumpVol_PostT1));
						NH4_NH3_N_Free[i] = NH4_Ammonium / NH4_Alpha_T1;
					}

					DEBUG_PRINT(UARTprintf("pH Cr:\t=%d/1000\t=%d/1000\n", (int) (pH_Samp_T1[ISEs.pH_Cr.index] * 1000), (int) (pH_Samp_T1[ISEs.pH_Cr.index + 1] * 1000));)
					DEBUG_PRINT(UARTprintf("Chose Cr %d with a pH of %d/1000\n", T_Chosen_pH_T1, (int) (pH_Samp_T1[T_Chosen_pH_T1 + ISEs.pH_Cr.index] * 1000));)
					DEBUG_PRINT(UARTprintf("NH4 of mixed T1:\t=%d/1000\t=%d/1000\n\n", (int) (NH4_NH3_N_Free[0] * 1000), (int) (NH4_NH3_N_Free[1] * 1000));)

#ifndef UNIVERSAL_PICKING_FUNCTION
					T_Chosen_NH4 = Choose_NH4_Sensor(Cal_Number, NH4_NH3_N_Free, NH4_E_Rinse, ISEs, Sols);
#else	// UNIVERSAL_PICKING_FUNCTION
					T_Chosen_NH4 = Choose_Sensor(Cal_Number, NH4_NH3_N_Free, NH4_E_Rinse, T_Rinse, ISEs.NH4, Sols);
#endif	// UNIVERSAL_PICKING_FUNCTION


					if(NH4_NH3_N_Free[T_Chosen_NH4] < 0)
						NH4_NH3_N_Free[T_Chosen_NH4] = 0;
					if(Cond_Cal_Status && ISE_Cal_Status[ISEs.NH4.index + T_Chosen_NH4] && ISE_Cal_Status[ISEs.pH_Cr.index + T_Chosen_pH] && ISEs.NH4.size > 0 && ISEs.pH_Cr.size > 0)
						MemoryWrite(Test_page, OFFSET_TEST_TOTAL_NH4, 4, (uint8_t *) &NH4_NH3_N_Free[T_Chosen_NH4]);

					float NH4_alpha_Therm = pow(10, -pH_Cr_Samp[T_Chosen_pH]) / (pow(10, -pH_Cr_Samp[T_Chosen_pH]) + pow(10, -(0.09018 + 2729.92/T_Therm)));
					NH4_Ammonium = NH4_NH3_N_Free[T_Chosen_NH4] * NH4_alpha_Therm;
					if(NH4_Ammonium < 0)
						NH4_Ammonium = 0;
					if(Cond_Cal_Status && ISE_Cal_Status[ISEs.NH4.index + T_Chosen_NH4] && ISE_Cal_Status[ISEs.pH_Cr.index + T_Chosen_pH] && ISEs.NH4.size > 0)
						MemoryWrite(Test_page, OFFSET_TEST_FREE_NH4, 4, (uint8_t *) &NH4_Ammonium);

					for(i = 0; i < 10; i++)
						MemoryWrite(Test_page, OFFSET_RAW_ISE_1_SAMP + (i * 4), 4, (uint8_t *) &ISE_E_Samp[i]);

					MemoryWrite(Test_page, OFFSET_NH4_T1_MIX_PH, 4, (uint8_t *) &pH_Samp_T1[T_Chosen_pH_T1 + ISEs.pH_Cr.index]);
					MemoryWrite(Test_page, OFFSET_NH4_T1_MIX_VOL, 4, (uint8_t *) &Volume_T1_End);
					MemoryWrite(Test_page, OFFSET_NH4_T1_MIX_COND, 4, (uint8_t *) &Conductivity_T1);

					update_Test(Test_Number);

					// For disinfection cartridge flush with sample to clear our HEPES
					if(ISEs.Config >= DISINFECTION_CART && ISEs.Config <= DISINFECTION_CART_2CR_6NH4_2CR)	// This is a disinfection cartridge
					{
						DEBUG_PRINT(UARTprintf("Rinsing with sample and air bubbles\n");)

						for (i = 0; i < 3; i++) // Loop over air/solution cycle 3 times for single solution
						{
							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
							PumpVolume(FW, PumpVol_Solution, Speed_Fast, 1);
							if(i != (Number_of_bubbles_samp - 1))
								userDelay(valve_delay, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
							PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 1);
							userDelay(valve_delay_after_air, 1);
						}
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
						PumpVolume(FW, PumpVol_sample_rinse, Speed_Fast, 1);
						userDelay(valve_delay, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
						PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 1);
						userDelay(valve_delay_after_air, 1);
					}
				}
			}
#endif	// STRAIGHT_TO_CL

#ifdef TESTING_MODE
			uint64_t NH4_mix_clock = TimerValueGet64(WTIMER0_BASE);
			DEBUG_PRINT(UARTprintf("Time to NH4 mix: %d\n", (uint32_t) ((NH4_mix_clock - alk_clock) / SysCtlClockGet()));)
			DEBUG_PRINT(UARTprintf("Total time elapsed: %d s, or %d m %d s\n", (uint32_t) ((NH4_mix_clock - start_clock)/SysCtlClockGet()), (uint32_t) ((NH4_mix_clock - start_clock)/SysCtlClockGet())/60,(uint32_t) ((NH4_mix_clock - start_clock)/SysCtlClockGet())%60);)
#endif

			//
			// Clean amperometrics in rinse
			//
			PrintTime();
			if((gui32Error & ABORT_ERRORS) == 0)
				if(CLEAN_AMPS_TEST == 1)
				{
//					if((ACID_AMP_CLEAN || BIAS_IN_NITRIC) && (gui32Error & ABORT_ERRORS) == 0)
//					{
//						// Prime a little T1 before test to clear out any contamination
//						if(BIAS_IN_NITRIC)
//						{
//							DEBUG_PRINT(UARTprintf("Biasing electrodes in acid...\n");)
//							// Pump in rinse over amperometrics for cleaning
//							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
//							FindPossitionZeroPump();
//							userDelay(valve_delay_after_air, 1);
//							RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);
//
//							if(BUBBLES_IN_TUBE)
//								PumpStepperRunStepSpeed_AbortReady(FW, Steps_tube_prime, Speed_ISE);
//							PumpStepperRunStepSpeed_AbortReady(FW, runSteps_Clean, Speed_ISE);
//							userDelay(valve_delay, 1);
//							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
//							PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble + Steps_tube_bubble, Speed_ISE);
//							userDelay(valve_delay_after_air, 1);
//							RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);
//							PumpStepperRunStepSpeed_AbortReady(BW, Steps_tube_bubble, Speed_ISE);
//							userDelay(valve_delay, 1);
//							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
//							PumpStepperRunStepSpeed_AbortReady(FW, runSteps_Clean_center, Speed_ISE);
//
//							SleepValve();
//
//							// Turn on short and off parallel resistor to allow large current flows
//							IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 1);	// Parallel switch must be on with short switch to work
//							IO_Ext_Set(IO_EXT2_ADDR, 2, WORK_EL_SHORT, 1);
//							if(HIGH_RANGE)
//								IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_MID_CURRENT, 1);
//
//							// Set reference for amperometric mode
//							IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
//							IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);
//
//							// Connect all electrodes together for measuring
//							IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 1);
//							IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);
//
//							DACVoltageSet(0, -200, true);
//
//							userDelay(20000, 1);		// Let run 20 seconds with short to allow any current necessary
//
//							IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 0);
//							IO_Ext_Set(IO_EXT2_ADDR, 2, WORK_EL_SHORT, 0);	// Turn off short switch
//
//							// Let the working electrodes float when not measuring
//							IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
//							IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 0);
//							if(HIGH_RANGE)
//								IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_MID_CURRENT, 0);
//							DACVoltageSet(0, 0, true);
//
//							// RE and CE floating
//							IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);		// Leave RE floating
//							IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);		// Leave CE floating
//						}
//						else
//						{
//							DEBUG_PRINT(UARTprintf("Pushing a little acid past arrays before cleaning... \n");)
//							RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);
//							PumpStepperRunStepSpeed_AbortReady(FW, Steps_tube_prime_buffers * 2, Speed_Metering);
//							userDelay(valve_delay, 1);
//
//							// Push a little bubble back into T1 tube to help prevent contamination
//							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
//							PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble + Steps_tube_bubble, Speed_placing);
//							userDelay(valve_delay_after_air, 1);
//							RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to sample
//							PumpStepperRunStepSpeed_AbortReady(BW, Steps_tube_bubble, Speed_placing);
//							userDelay(valve_delay, 1);
//							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
//							PumpStepperRunStepSpeed_AbortReady(FW, 15000, Speed_Metering);
//							userDelay(valve_delay_after_air, 1);
//						}
//					}

#ifdef BLEACH_POUCH
					// Pump in clean over amperometrics for cleaning
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
					FindPossitionZeroPump();
					userDelay(valve_delay_after_air, 1);
					DEBUG_PRINT(UARTprintf("Pumping bleach!\n");)
					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					PumpVolume(FW, PumpVol_tube_bubble + PumpVol_Solution, Speed_Fast, 1);
					userDelay(valve_delay, 1);
					RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);
					PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_Fast, 1);
					userDelay(valve_delay, 1);

					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					PumpVolume(FW, PumpVol_sample_rinse, Speed_Fast, 1);
					userDelay(valve_delay, 1);

					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					PumpVolume(FW, PumpVol_air_bubble + PumpVol_tube_bubble, Speed_Fast, 1);
					userDelay(valve_delay_after_air, 1);
					RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);
					PumpVolume(BW, PumpVol_tube_bubble, Speed_Fast, 1);
					userDelay(valve_delay, 1);

					SleepValve();
#endif

					if(ISEs.Config == PH_CL_CART)
					{
						// Pump like we would for Prerinse to clear out chip and get a reliable reading
						PrintTime();
						update_Status(STATUS_TEST, OPERATION_CL_ACTIVATION);

						if((gui32Error & ABORT_ERRORS) == 0)
						{
#ifdef CAL_2_RINSE
							DEBUG_PRINT(UARTprintf("Pumping Cal 2 as prerinse... \n");)
#else
							DEBUG_PRINT(UARTprintf("Pumping prerinse... \n");)
#endif

							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
							FindPossitionZeroPump();
							uint16_t PumpSpeed = Speed_Fast;
							for (i = 0; i < Number_of_bubbles_Prerinse; i++) // Loop over air/solution cycle 3 times for single solution
							{
								RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
								PumpVolume(FW, PumpVol_air_bubble, PumpSpeed, 1);
								if(i == 2)
									PumpSpeed = Speed_Metering;
								if(i == (Number_of_bubbles_Prerinse - 1))
									PumpVolume(FW, PumpVol_Large_air_bubble, PumpSpeed, 1);
								userDelay(valve_delay_after_air, 1);
#ifdef CAL_2_RINSE
								RunValveToPossition_Bidirectional_AbortReady(V_CAL_2, VALVE_STEPS_PER_POSITION);
#else
								RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);
#endif
								if(i == 0 && BUBBLES_IN_TUBE)
									PumpVolume(FW, PumpVol_tube_bubble, PumpSpeed, 1);
								PumpVolume(FW, PumpVol_Solution, PumpSpeed, 1);
								if(i != (Number_of_bubbles_Prerinse - 1))
									userDelay(valve_delay, 1);
							}

#ifdef PUMP_CLEAN
							PumpStepperRunStepSpeed_AbortReady(FW, runSteps_PreRinse + runSteps_plug, Speed_ISE);
							userDelay(valve_delay, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge

							SleepValve();

#else
							PumpVolume(FW, PumpVol_Rinse, PumpSpeed, 1);
							userDelay(valve_delay, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
							PumpVolume(FW, PumpVol_plug, PumpSpeed, 1);

							SleepValve();
#endif

							if((gui32Error & ABORT_ERRORS) == 0)
								Sensor_in_rinse = 1;
						}

						CollectISEmV(ISE_E_Rinse, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);

						T_Rinse = MeasureTemperature(1);

						// Set RE and CE floating and close RE/CE loop
						IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
						IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);

//						Conductivity_Rinse = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);
#ifndef COND_SOLUTION_STRUCT
						if(Sols->pH_EEP_Cal_2 < 9) // This is Cal 3, not Cal 2
						{
							if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
								Conductivity_Rinse = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_1, 0);
							else
								Conductivity_Rinse = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_1, 0);
						}
						else
						{
							if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
								Conductivity_Rinse = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_2, 0);
							else
								Conductivity_Rinse = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);
						}
#else
						Conductivity_Rinse = MeasureConductivity(Sols, 0);
#endif

						DEBUG_PRINT(UARTprintf("Clean Conductivity Measured at: %d\n", (int) (Conductivity_Rinse * 1000));)
						Conductivity_Rinse /= (1 + Sols->Clean_Cond_TComp*(T_Rinse - 25));
						DEBUG_PRINT(UARTprintf("Rinse temp and corrected conductivity: =%d/1000\t=%d/1000\n", (int) (T_Rinse * 1000), (int) ((Conductivity_Rinse) * 1000));)

						DEBUG_PRINT(UARTprintf("Clean Conductivity saved on memory: %d\n", (int) (Sols->Cond_EEP_Clean * 1000));)

						MemoryWrite(Test_page, OFFSET_RAW_T_RINSE, 4, (uint8_t *) &T_Rinse);
						MemoryWrite(Test_page, OFFSET_TEST_CLEAN_COND_TCOR, 4, (uint8_t *) &Conductivity_Rinse);
						for(i = 0; i < 10; i++)
							MemoryWrite(Test_page, OFFSET_RAW_ISE_1_RINSE + (i * 4), 4, (uint8_t *) &ISE_E_Rinse[i]);

						if(REF_DRIFT != 0 && SATURATED_KCL_REF != 0)
							Ref_drift = Calculate_Ref_Drift(SATURATED_KCL_REF, T_Rinse);

						if(ISEs.Config == PH_CL_CART)
						{
							//
							// pH Measurement
							//
							//					pH_TCor_Rinse = pH_EEP_Rinse + K_T_pH_Rinse * (T_RS - 25);	// Temperature corrected pH for Rinse
							T_RS = (T_Rinse + T_Samp) / 2;
#ifdef CAL_2_RINSE
							pH_TCor_Rinse = Calc_pH_TCor(Sols->pH_EEP_Cal_2, T_RS, 25, 0, Sols->K_T_pH_Cal_2);
#else
							pH_TCor_Rinse = Calc_pH_TCor(Sols->pH_EEP_Clean, T_RS, 25, Sols->K_T_pH_Clean_Sq, Sols->K_T_pH_Clean_Ln);
#endif

							for(i = 0; i < ISEs.pH_Cr.size; i++)
							{
								float pH_Cr_Slope_RST = pH_Cr_EEP_Slope[i] * (T_RS + 273.0) / (T_EEP_Cal + 273.0);	// Temperature corrected slope
								// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
								float pH_Cr_Samp_RS = pH_TCor_Rinse + ((pH_Cr_E_Samp[i] - pH_Cr_E_Rinse[i]) / pH_Cr_Slope_RST); // pH of sample
								if(T_Therm > 2 && T_Therm < 50)
								{
									pH_Cr_Samp[i] = Calc_pH_TCor(pH_Cr_Samp_RS, T_Therm, T_RS, K_T_pH_Samp_Sq, K_T_pH_Samp_Ln);
									DEBUG_PRINT(UARTprintf("pH Cr %d, Uncorrected: %d, Corrected: %d\n", i + 1, (int) (pH_Cr_Samp_RS * 1000), (int) (pH_Cr_Samp[i] * 1000));)
								}
								else
								{
									//							DEBUG_PRINT(UARTprintf("Thermistor temp read outside acceptable range, not adjusting pH!\n");)
									pH_Cr_Samp[i] = pH_Cr_Samp_RS;
									gui32Error |= THERMISTOR_FAILED;
								}
							}

							if(T_Therm < 2 || T_Therm > 50)
								{DEBUG_PRINT(UARTprintf("Thermistor temp read outside acceptable range, not adjusting pH!\n");)}


#ifndef UNIVERSAL_PICKING_FUNCTION
							T_Chosen_pH = Choose_pH_Sensor_pHDie(Cal_Number, ISE_Reading);
#else	// UNIVERSAL_PICKING_FUNCTION
							T_Chosen_pH = Choose_Sensor(Cal_Number, pH_Cr_Samp, pH_Cr_E_Rinse, T_Rinse, ISEs.pH_Cr, Sols);
#endif	// UNIVERSAL_PICKING_FUNCTION

							if(ISE_Cal_Status[ISEs.pH_Cr.index + T_Chosen_pH] && ISEs.pH_Cr.size > 0)	// Check that chosen sensor passed calibration before reporting a number
								MemoryWrite(Test_page, OFFSET_TEST_PH, 4, (uint8_t *) &pH_Cr_Samp[T_Chosen_pH]);

							//
							// Conductivity Temperature Correction
							//
							// Perform temperature correction here after calculations for ISEs so we are using the conductivity at temperature, not the adjusted conductivity
							Conductivity /= (1 + COND_TCOMP_SAMP*(T_Samp - 25));

							//
							ui8TChosen_Sensors = T_Chosen_pH;
							MemoryWrite(Test_page, OFFSET_CHOSEN_SENSORS, 1, &ui8TChosen_Sensors);

							if(Cond_Cal_Status)
								MemoryWrite(Test_page, OFFSET_TEST_COND, 4, (uint8_t *) &Conductivity);

							update_Test(Test_Number);
						}

						if(MEASURE_FCL || MEASURE_TCL)
						{
#ifndef CV_CLEANING
//							if(gABoard >= AV7_3)
//								CleanAmperometrics_CurrentLimited(Ref_drift, 0, Test_Number, OXIDE_REBUILD_TYPE);
//							else
//								CleanAmperometrics(Ref_drift, 0, Test_Number, OXIDE_REBUILD_TYPE);

#ifdef CC_CURRENT_LIMITED
							CleanAmperometrics_CurrentLimited(Ref_drift, 0, Test_Number, OXIDE_REBUILD_TYPE);
#else
							CleanAmperometrics(Ref_drift, 0, Test_Number, OXIDE_REBUILD_TYPE);
#endif	// CC_CURRENT_LIMITED

#else
							RunCVCleaningCycle(Ref_drift, 0, Test_Number);
#endif

							uint8_t App_Cal_Status = *MemoryRead(Cal_page, OFFSET_CALIBRATED_STATUS, 1);
							if((gui32Error & CL_CLEANING_OUT_OF_RANGE) != 0)	// Cl Cleaning failed
								App_Cal_Status &= ~0x80;	// Clear the Cl Cleaning flag
							else
								App_Cal_Status |= 0x80;		// Set the Cl Cleaning flag

							MemoryWrite(Cal_page, OFFSET_CALIBRATED_STATUS, 1, &App_Cal_Status);	// Write the new Cl flag to memory

							update_Test(Test_Number);
						}

						// Push air back into rinse port before moving to next solution
						if(BUBBLES_IN_TUBE)
						{
#ifdef CAL_2_RINSE
							RunValveToPossition_Bidirectional_AbortReady(V_CAL_2, VALVE_STEPS_PER_POSITION);
#else
							RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);
#endif
							PumpVolume(BW, PumpVol_tube_bubble, Speed_Fast, 1);
							userDelay(valve_delay, 1);
						}
					}
					else if(MEASURE_FCL || MEASURE_TCL)
					{
						update_Status(STATUS_TEST, OPERATION_CL_ACTIVATION);

						// Pump in clean over amperometrics for cleaning
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
						FindPossitionZeroPump();
						userDelay(valve_delay_after_air, 1);
						DEBUG_PRINT(UARTprintf("Pumping cleaning solution to clean amperometrics!\n");)
						RunValveToPossition_Bidirectional_AbortReady(V_CLEAN, VALVE_STEPS_PER_POSITION);

#ifdef PUMP_CLEAN
						if(BUBBLES_IN_TUBE)
							PumpStepperRunStepSpeed_AbortReady(FW, Steps_tube_prime, Speed_ISE);
						PumpStepperRunStepSpeed_AbortReady(FW, runSteps_Clean + runSteps_Clean_center + Steps_tube_bubble, Speed_ISE);
						userDelay(valve_delay, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						SleepValve();

#else
						if(BUBBLES_IN_TUBE)
							PumpVolume(FW, PumpVol_tube_bubble, Speed_Fast, 1);
						PumpVolume(FW, PumpVol_Clean, Speed_Fast, 1);
						userDelay(valve_delay, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						PumpVolume(FW, PumpVol_air_bubble + PumpVol_tube_bubble, Speed_Fast, 1);
						userDelay(valve_delay_after_air, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_CLEAN, VALVE_STEPS_PER_POSITION);
						PumpVolume(BW, PumpVol_tube_bubble, Speed_Fast, 1);
						userDelay(valve_delay, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						PumpVolume(FW, PumpVol_Clean_center, Speed_Fast, 1);

						SleepValve();
#endif

						// Set RE and CE floating and close RE/CE loop
						IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
						IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);

//						Conductivity_Rinse = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);
#ifndef COND_SOLUTION_STRUCT
						if(Sols->pH_EEP_Cal_2 < 9) // This is Cal 3, not Cal 2
						{
							if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
								Conductivity_Rinse = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_1, 0);
							else
								Conductivity_Rinse = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_1, 0);
						}
						else
						{
							if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
								Conductivity_Rinse = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_2, 0);
							else
								Conductivity_Rinse = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);
						}
#else
						Conductivity_Rinse = MeasureConductivity(Sols, 0);
#endif

//						DEBUG_PRINT(UARTprintf("Clean Conductivity Measured at: %d\n", (int) (Conductivity_Rinse * 1000));)
//						if(ISEs.Config == PH_CL_CART)
//							DEBUG_PRINT(UARTprintf("Clean temp and corrected conductivity: =%d/1000\t=%d/1000\n", (int) (T_Rinse * 1000), (int) ((Conductivity_Rinse / (1 + Sols->Clean_Cond_TComp*(T_Rinse - 25))) * 1000));)
//						else
//							DEBUG_PRINT(UARTprintf("Clean temp and corrected conductivity: =%d/1000\t=%d/1000\n", (int) (T_Rinse * 1000), (int) ((Conductivity_Rinse / (1 + Sols->Rinse_Cond_TComp*(T_Rinse - 25))) * 1000));)
//						DEBUG_PRINT(UARTprintf("Clean Conductivity saved on memory: %d\n", (int) (Sols->Cond_EEP_Rinse * 1000));)

						//						Conductivity_Rinse = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);

						DEBUG_PRINT(UARTprintf("Clean Conductivity Measured at: %d\n", (int) (Conductivity_Rinse * 1000));)
						Conductivity_Rinse /= (1 + Sols->Clean_Cond_TComp*(T_Rinse - 25));
						DEBUG_PRINT(UARTprintf("Clean temp and corrected conductivity: =%d/1000\t=%d/1000\n", (int) (T_Rinse * 1000), (int) ((Conductivity_Rinse) * 1000));)
						DEBUG_PRINT(UARTprintf("Clean Conductivity saved on memory: %d\n", (int) (Sols->Cond_EEP_Clean * 1000));)

						MemoryWrite(Test_page, OFFSET_TEST_CLEAN_COND_TCOR, 4, (uint8_t *) &Conductivity_Rinse);

#ifdef SINGLE_POINT_H2_IN_CLEAN
						if(MEASURE_ALKALINITY && ISEs.RunAlk)
						{
							float ISE_E_Clean[10];
							CollectISEmV(ISE_E_Clean, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);
							float * pH_H2_E_Clean = &ISE_E_Clean[ISEs.pH_H2.index];
							float * pH_Cr_E_Clean = &ISE_E_Clean[ISEs.pH_Cr.index];

							float T_Clean = MeasureTemperature(1);

							// Set RE and CE floating and close RE/CE loop
							IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
							IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);

							float Conductivity_Clean = MeasureConductivity(Cond_EEP_Rinse, Cond_EEP_Cal_2, 0);
							DEBUG_PRINT(UARTprintf("Clean Conductivity Measured at: %d\n", (int) (Conductivity_Clean * 1000));)
							DEBUG_PRINT(UARTprintf("Clean temp and corrected conductivity: =%d/1000\t=%d/1000\n", (int) (T_Clean * 1000), (int) ((Conductivity_Clean / (1 + Sols->Clean_Cond_TComp*(T_Clean - 25))) * 1000));)
	//						float Cond_EEP_Clean = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_C))
	//						DEBUG_PRINT(UARTprintf("Clean Conductivity saved on memory: %d\n", (int) (Cond_EEP_Clean * 1000));)

	//						MemoryWrite(Test_page, OFFSET_RAW_T_RINSE, 4, (uint8_t *) &T_Rinse);
	//						for(i = 0; i < 10; i++)
	//							MemoryWrite(Test_page, OFFSET_RAW_ISE_1_RINSE + (i * 4), 4, (uint8_t *) &ISE_E_Rinse[i]);

							//
							// pH H2 Measurement
							//
							float pH_H2_Clean_RS[2];
							float pH_H2_Slope_RST[2];
							for(i = 0; i < ISEs.pH_H2.size; i++)
							{
								pH_H2_Slope_RST[i] = pH_H2_EEP_Slope[i] * (T_RS + 273.0) / (T_EEP_Cal + 273.0);	// Temperature corrected slope
								// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
								float pH_H2_Clean_TClean = pH_TCor_Rinse + ((pH_H2_E_Clean[i] - pH_H2_E_Rinse[i]) / pH_H2_Slope_RST[i]); // pH of sample
								if(T_Therm > 2 && T_Therm < 50)
								{
									pH_H2_Clean_RS[i] = Calc_pH_TCor(pH_H2_Clean_TClean, T_RS, T_Clean, K_T_pH_Clean_Sq, K_T_pH_Clean_Ln);
									DEBUG_PRINT(UARTprintf("Clean pH H2 %d, Uncorrected:\t%d\tCorrected to 25:\t%d\n", i + 1, (int) (pH_H2_Clean_TClean * 1000), (int) (pH_H2_Clean_RS[i] * 1000));)
								}
								else
								{
									DEBUG_PRINT(UARTprintf("Clean pH H2 %d, Uncorrected:\t%d\n", i + 1, (int) (pH_H2_Clean_TClean * 1000));)
								}
							}

							//
							// pH Cr Measurement
							//
							float pH_Cr_Clean_RS[2];
							float pH_Cr_Slope_RST[2];
							for(i = 0; i < ISEs.pH_Cr.size; i++)
							{
								pH_Cr_Slope_RST[i] = pH_Cr_EEP_Slope[i] * (T_RS + 273.0) / (T_EEP_Cal + 273.0);	// Temperature corrected slope
								// Removed temperature correction for sample because we are assuming T_Rinse = T_Samp
								float pH_Cr_Clean_TClean = pH_TCor_Rinse + ((pH_Cr_E_Clean[i] - pH_Cr_E_Rinse[i]) / pH_Cr_Slope_RST[i]); // pH of sample
								if(T_Therm > 2 && T_Therm < 50)
								{
									pH_Cr_Clean_RS[i] = Calc_pH_TCor(pH_Cr_Clean_TClean, T_RS, T_Clean, K_T_pH_Clean_Sq, K_T_pH_Clean_Ln);
									DEBUG_PRINT(UARTprintf("Clean pH Cr %d, Uncorrected:\t%d\tCorrected to 25:\t%d\n", i + 1, (int) (pH_Cr_Clean_TClean * 1000), (int) (pH_Cr_Clean_RS[i] * 1000));)
								}
								else
								{
									DEBUG_PRINT(UARTprintf("Clean pH Cr %d, Uncorrected:\t%d\n", i + 1, (int) (pH_Cr_Clean_TClean * 1000));)
								}
							}
							DEBUG_PRINT(UARTprintf("\n");)

							//
							// Recalculate Alkalinity based on these pH values
							//
							uint16_t Steps_T1[2];
							Steps_T1[0] = *((uint16_t *) MemoryRead(Test_page, OFFSET_STEPS_T1_1, 2));
							Steps_T1[1] = *((uint16_t *) MemoryRead(Test_page, OFFSET_STEPS_T1_2, 2));

							// Read what the samples were calculated as from memory
							float pH_Samp_T1[20];
							pH_Samp_T1[0] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_1_T1_1, 4));
							pH_Samp_T1[10] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_1_T1_2, 4));
							pH_Samp_T1[1] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_2_T1_1, 4));
							pH_Samp_T1[11] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_2_T1_2, 4));
							pH_Samp_T1[2] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_3_T1_1, 4));
							pH_Samp_T1[12] = Build_float(MemoryRead(Test_page, OFFSET_TEST_PH_3_T1_2, 4));

							for(i = 0; i < 7; i++)
							{
								pH_Samp_T1[3 + i] = Build_float(MemoryRead(Test_page, OFFSET_ISE_4_T1_1 + (i * 4), 4));
								pH_Samp_T1[13 + i] = Build_float(MemoryRead(Test_page, OFFSET_ISE_4_T1_2 + (i * 4), 4));
							}

							// Calculate what the pH would be if using Clean as single point offset
							float pH_TCor_Clean = Calc_pH_TCor(pH_EEP_Clean, T_RS, 25, K_T_pH_Clean_Sq, K_T_pH_Clean_Ln);
							for(i = 0; i < ISEs.pH_H2.size; i++)
							{
								if(pH_Samp_T1[i + ISEs.pH_H2.index] != 0)	// Don't recalculate if it is 0, this means it didn't read, for second point it could be because it found endpoint
									pH_Samp_T1[i + ISEs.pH_H2.index] += (pH_TCor_Clean - pH_TCor_Rinse) + (pH_H2_E_Rinse[i] - pH_H2_E_Clean[i]) / pH_H2_Slope_RST[i];
								if(pH_Samp_T1[i + ISEs.pH_H2.index + 10] != 0)	// Don't recalculate if it is 0, this means it didn't read, for second point it could be because it found endpoint
									pH_Samp_T1[i + ISEs.pH_H2.index + 10] += (pH_TCor_Clean - pH_TCor_Rinse) + (pH_H2_E_Rinse[i] - pH_H2_E_Clean[i]) / pH_H2_Slope_RST[i];
							}
							for(i = 0; i < ISEs.pH_Cr.size; i++)
							{
								if(pH_Samp_T1[i + ISEs.pH_Cr.index] != 0)	// Don't recalculate if it is 0, this means it didn't read, for second point it could be because it found endpoint
									pH_Samp_T1[i + ISEs.pH_Cr.index] += (pH_TCor_Clean - pH_TCor_Rinse) + (pH_Cr_E_Rinse[i] - pH_Cr_E_Clean[i]) / pH_Cr_Slope_RST[i];
								if(pH_Samp_T1[i + ISEs.pH_Cr.index + 10] != 0)	// Don't recalculate if it is 0, this means it didn't read, for second point it could be because it found endpoint
								pH_Samp_T1[i + ISEs.pH_Cr.index + 10] += (pH_TCor_Clean - pH_TCor_Rinse) + (pH_Cr_E_Rinse[i] - pH_Cr_E_Clean[i]) / pH_Cr_Slope_RST[i];
							}

							// Print out the newly calculated pH T1 mix values using clean as the single point offset
							DEBUG_PRINT(UARTprintf("Recalculated pH of first T1 mix:\n");)
							DEBUG_PRINT(UARTprintf("pH H2:\t%d\t%d\n", (int) (pH_Samp_T1[ISEs.pH_H2.index] * 1000), (int) (pH_Samp_T1[ISEs.pH_H2.index + 1] * 1000));)
							DEBUG_PRINT(UARTprintf("pH Cr:\t%d\t%d\n\n", (int) (pH_Samp_T1[ISEs.pH_Cr.index] * 1000), (int) (pH_Samp_T1[ISEs.pH_Cr.index + 1] * 1000));)

							DEBUG_PRINT(UARTprintf("Recalculated pH of second T1 mix:\n");)
							DEBUG_PRINT(UARTprintf("pH H2:\t%d\t%d\n", (int) (pH_Samp_T1[ISEs.pH_H2.index + 10] * 1000), (int) (pH_Samp_T1[ISEs.pH_H2.index + 11] * 1000));)
							DEBUG_PRINT(UARTprintf("pH Cr:\t%d\t%d\n\n", (int) (pH_Samp_T1[ISEs.pH_Cr.index + 10] * 1000), (int) (pH_Samp_T1[ISEs.pH_Cr.index + 11] * 1000));)

	//						float T_Samp_T1[2];
	//						T_Samp_T1[0] = Build_float(MemoryRead(Test_page, OFFSET_RAW_T_SAMP_T1_1, 4));
	//						T_Samp_T1[1] = Build_float(MemoryRead(Test_page, OFFSET_RAW_T_SAMP_T1_2, 4));

							// Calculate Steps endpoint given these pHs
							// Alkalinity
	//						float Alk_Samp[10] = {nanf(), nanf(), nanf(), nanf(), nanf(), nanf(), nanf(), nanf(), nanf(), nanf()};
							float Alk_Slope[10] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0};
							uint8_t method[10] = {0,0,0,0,0,0,0,0,0,0};

							uint8_t Alk_index = 0;
							DEBUG_PRINT(UARTprintf("Recalculated Alkalinity using Clean:\n");)
							for(Alk_index = 0; Alk_index < (ISEs.pH_H2.size + ISEs.pH_Cr.size); Alk_index++)
							{
								float Steps_Sample = 7000;	// First mixing, Second mixing
								float Pump_Ratio = 0.61;	// TODO: Adjust pump ratio for alkalinity here
								float Steps_Samp_Endpoint = Steps_Sample * Pump_Ratio;
								float Steps_T1_Endpoint = 0;
								float pH_T1[2];

								if(Alk_index < ISEs.pH_H2.size)
								{
									pH_T1[0] = pH_Samp_T1[ISEs.pH_H2.index + (Alk_index)];
									pH_T1[1] = pH_Samp_T1[ISEs.pH_H2.index + (Alk_index + 10)];
								}
								else
								{
									pH_T1[0] = pH_Samp_T1[ISEs.pH_Cr.index + (Alk_index - ISEs.pH_H2.size)];
									pH_T1[1] = pH_Samp_T1[ISEs.pH_Cr.index + 10 + (Alk_index - ISEs.pH_H2.size)];
								}

								// First check if we found the endpoint on either mix, then pick the mix that was closest to 4.5
								if(((pH_T1[0] >= 4.1 && pH_T1[0] <= 4.9) || (pH_T1[1] >= 4.1 && pH_T1[1] <= 4.9)) && ISE_Cal_Status[Alk_index + ISEs.pH_H2.index])
								{
									if(abs_val(pH_T1[0] - 4.5) < abs_val(pH_T1[1] - 4.5))
										Steps_T1_Endpoint = Steps_T1[0];
									else
										Steps_T1_Endpoint = Steps_T1[1];
									method[Alk_index] = 2;
								}

								if(Steps_T1_Endpoint == 0 && ISE_Cal_Status[Alk_index + ISEs.pH_H2.index])	// Calculate endpoint if pH sensor passed calibration and hasn't already found endpoint
								{
									if(pH_T1[0] < 4.1 && pH_T1[1] < 4.1 && pH_T1[1] != 0)	// Both pH points landed below 4.1 endpoint, and there is saved data
									{
										Alk_Slope[Alk_index] = ((((float) Steps_Samp_Endpoint + (float) Steps_T1[1]) * pow(10, -pH_T1[1])) - (((float) Steps_Samp_Endpoint + (float) Steps_T1[0]) * pow(10, -pH_T1[0]))) / ((float) Steps_T1[1] - (float) Steps_T1[0]);
										float B = (((float) Steps_Samp_Endpoint + (float) Steps_T1[0]) * pow(10, -pH_T1[0])) - (Alk_Slope[Alk_index] * (float) Steps_T1[0]);
										Steps_T1_Endpoint = -B/Alk_Slope[Alk_index];
										method[Alk_index] = 3;
									}
									else if(pH_T1[0] < 4.1)
									{
										Steps_T1_Endpoint = ((float) Steps_T1[0] * (HCl_N - pow(10, -pH_T1[0])) + Pump_Ratio * Steps_Sample * (pow(10, -4.5) - pow(10, -pH_T1[0]))) / (HCl_N - pow(10, -4.5));
										method[Alk_index] = 5;
									}
								}

	//							float Alk_Saved = Build_float(MemoryRead(Test_page, OFFSET_TEST_ALKALINITY, 4));
	//							if(Steps_T1_Endpoint == 0 && Alk_Saved != -1 && pH_T1[0] <= 4.9)	// First two methods didn't find endpoint but during test did find one, meaning calculated based on steps being close enough
	//							{
	//								Alk_Samp[Alk_index] = Alk_Saved;
	//								method[Alk_index] = 5;
	//							}

								if(Steps_T1_Endpoint != 0)
								{
									Alk_Samp[Alk_index] = 50044.0 * HCl_N * Steps_T1_Endpoint / Steps_Samp_Endpoint;
								}

								if(Alk_index < ISEs.pH_H2.size)
								{
									DEBUG_PRINT(UARTprintf("pH H2 %d:\t%d\t", Alk_index + 1, (int) (Alk_Samp[Alk_index] * 1000));)
								}
								else
								{
									DEBUG_PRINT(UARTprintf("pH Cr %d:\t%d\t", Alk_index + 1 - ISEs.pH_H2.size, (int) (Alk_Samp[Alk_index] * 1000));)
								}



								DEBUG_PRINT(UARTprintf("=%d/1000\t", (int) (Alk_Samp[Alk_index] * 1000));)
								if(method[Alk_index] == 2)
									{DEBUG_PRINT(UARTprintf("Endpoint");)}
								else if(method[Alk_index] == 3)
									{DEBUG_PRINT(UARTprintf("Overshoot");)}
								else if(method[Alk_index] == 5)
									{DEBUG_PRINT(UARTprintf("Assumed Slope");)}
								else
									{DEBUG_PRINT(UARTprintf("Failed");)}
								DEBUG_PRINT(UARTprintf("\n");)
	//							DEBUG_PRINT(UARTprintf("=%d/1000\t", (int) (Alk_Slope[Alk_index] * 1000));)
							}

							// Choose alkalinity sensor again
							if(ISEs.pH_H2.size > 0)
								T_Chosen_Alk = Choose_Alk_Sensor(Cal_Number, Alk_Samp, pH_H2_E_Clean, T_Rinse, method, Alk_Slope, ISEs);
							else
								T_Chosen_Alk = Choose_Alk_Sensor(Cal_Number, Alk_Samp, pH_Cr_E_Rinse, T_Rinse, method, Alk_Slope, ISEs);

							DEBUG_PRINT(UARTprintf("Chosen Alk sensors (added 1): %d\n\n", (T_Chosen_Alk + 1));)

							for(i = 0; i < ISEs.pH_H2.size; i++)
								MemoryWrite(Test_page, OFFSET_RAW_ISE_1_RINSE + ((i + ISEs.pH_H2.index) * 4), 4, (uint8_t *) &ISE_E_Clean[i + ISEs.pH_H2.index]);

							ui8TChosen_Sensors &= ~(1 << ISEs.pH_H2.StorBit);	// Create a mask and set the chosen sensor to 0 so the new chosen sensor can be OR'd in
							ui8TChosen_Sensors |= T_Chosen_Alk << ISEs.pH_H2.StorBit;

							// Store data from alkalinity
							MemoryWrite(Test_page, OFFSET_TEST_PH_1_T1_1, 4, (uint8_t *) &pH_Samp_T1[0]);
							MemoryWrite(Test_page, OFFSET_TEST_PH_1_T1_2, 4, (uint8_t *) &pH_Samp_T1[10]);
							MemoryWrite(Test_page, OFFSET_TEST_PH_2_T1_1, 4, (uint8_t *) &pH_Samp_T1[1]);
							MemoryWrite(Test_page, OFFSET_TEST_PH_2_T1_2, 4, (uint8_t *) &pH_Samp_T1[11]);
							MemoryWrite(Test_page, OFFSET_TEST_PH_3_T1_1, 4, (uint8_t *) &pH_Samp_T1[2]);
							MemoryWrite(Test_page, OFFSET_TEST_PH_3_T1_2, 4, (uint8_t *) &pH_Samp_T1[12]);

							// Write the rest of the values to the memory
							for(i = 0; i < 7; i++)
							{
								MemoryWrite(Test_page, OFFSET_ISE_4_T1_1 + (i * 4), 4, (uint8_t *) &pH_Samp_T1[3 + i]);
								MemoryWrite(Test_page, OFFSET_ISE_4_T1_2 + (i * 4), 4, (uint8_t *) &pH_Samp_T1[13 + i]);
							}

	#ifndef TESTING_MODE
							if(Alk_Samp[T_Chosen_Alk] < 0)
								Alk_Samp[T_Chosen_Alk] = 0;
	#endif

							if(ISE_Cal_Status[ISEs.pH_H2.index + T_Chosen_Alk])
								MemoryWrite(Test_page, OFFSET_TEST_ALKALINITY, 4, (uint8_t *) &Alk_Samp[T_Chosen_Alk]);
						}
#endif	// SINGLE_POINT_H2_IN_CLEAN

						if(REF_DRIFT != 0 && SATURATED_KCL_REF != 0)
							Ref_drift = Calculate_Ref_Drift(SATURATED_KCL_REF, MeasureTemperature(1));

#ifdef MEASURE_NITRITE
						if(MEASURE_FCL == 0 && MEASURE_TCL == 0 && MEASURE_NITRITE)
#else
						if(MEASURE_FCL == 0 && MEASURE_TCL == 0)
#endif
						{
//							CleanAmperometrics_CurrentLimited(Ref_drift, 0, Test_Number, 0);

#ifdef CC_CURRENT_LIMITED
							CleanAmperometrics_CurrentLimited(Ref_drift, 0, Test_Number, OXIDE_REBUILD_TYPE);
#else
							CleanAmperometrics(Ref_drift, 0, Test_Number, OXIDE_REBUILD_TYPE);
#endif	// CC_CURRENT_LIMITED
						}
						else
						{
#ifndef CV_CLEANING
//							if(gABoard >= AV7_3)
//								CleanAmperometrics_CurrentLimited(Ref_drift, 0, Test_Number, OXIDE_REBUILD_TYPE);
//							else
//								CleanAmperometrics(Ref_drift, 0, Test_Number, OXIDE_REBUILD_TYPE);
#ifdef CC_CURRENT_LIMITED
							CleanAmperometrics_CurrentLimited(Ref_drift, 0, Test_Number, OXIDE_REBUILD_TYPE);
#else
							CleanAmperometrics(Ref_drift, 0, Test_Number, OXIDE_REBUILD_TYPE);
#endif	// CC_CURRENT_LIMITED

#else
							RunCVCleaningCycle(Ref_drift, 0, Test_Number);
#endif
						}

#ifdef PUMP_CLEAN
						RunValveToPossition_Bidirectional_AbortReady(V_CLEAN, VALVE_STEPS_PER_POSITION);
						PumpStepperRunStepSpeed_AbortReady(BW, Steps_tube_bubble, Speed_ISE);
						userDelay(valve_delay, 1);
#endif

						uint8_t App_Cal_Status = *MemoryRead(Cal_page, OFFSET_CALIBRATED_STATUS, 1);
						if((gui32Error & CL_CLEANING_OUT_OF_RANGE) != 0)	// Cl Cleaning failed
							App_Cal_Status &= ~0x80;	// Clear the Cl Cleaning flag
						else
							App_Cal_Status |= 0x80;		// Set the Cl Cleaning flag

						MemoryWrite(Cal_page, OFFSET_CALIBRATED_STATUS, 1, &App_Cal_Status);	// Write the new Cl flag to memory

						update_Test(Test_Number);
					}
				}

#ifdef TESTING_MODE
			uint64_t clean_clock = TimerValueGet64(WTIMER0_BASE);
			DEBUG_PRINT(UARTprintf("Time to clean: %d\n", (uint32_t) ((clean_clock - NH4_mix_clock) / SysCtlClockGet()));)
			DEBUG_PRINT(UARTprintf("Total time elapsed: %d s, or %d m %d s\n", (uint32_t) ((clean_clock - start_clock)/SysCtlClockGet()), (uint32_t) ((clean_clock - start_clock)/SysCtlClockGet())/60,(uint32_t) ((clean_clock - start_clock)/SysCtlClockGet())%60);)
#endif

			// Create variables for Cl mixing
			float Cl_nA_FCl = 0;
			float Cl_nA_TCl = 0;
			float Cl_FCl_ppm = nanf(""), Cl_TCl_ppm = nanf(""), Cl_MCl_ppm = nanf("");//, Cl_MCl_NH3 = -1;
			float NH4_TNH3 = nanf(""), NH4_Cl_NH3 = nanf(""), /*NH4_BFR = -1*//*Nitrification_Potential = nanf("");*/Nitrification_Capacity = nanf("");
			float T_Samp_B2 = T_assume;
			float T_Samp_B1 = T_assume;
			PrintTime();
			if(REF_DRIFT != 0 && SATURATED_KCL_REF != 0)
				Ref_drift = Calculate_Ref_Drift(SATURATED_KCL_REF, MeasureTemperature(1));
			float Amp_Voltage_Set = 470 - Ref_drift;
			DEBUG_PRINT(UARTprintf("Amperometric set voltage = %d mV\n", (int) (Amp_Voltage_Set));)
//			float Amp_Voltage_Set_FCl = 400 - Ref_drift;
//			DEBUG_PRINT(UARTprintf("Amperometric set voltage FCl = %d mV\n", (int) (Amp_Voltage_Set_FCl));)
//			DEBUG_PRINT(UARTprintf("Amperometric set voltage TCl = %d mV\n", (int) (Amp_Voltage_Set));)


#ifdef PRIME_BUFFERS_TEST
			//
			// Prime B2, Testing here before B1 because everywhere else it could cause significantly worse issues
			//
			PrintTime();
			if((gui32Error & (ABORT_ERRORS | CL_CLEANING_OUT_OF_RANGE)) == 0)
				if(MEASURE_TCL)
				{
					uint8_t priming_index = 0;
//					uint8_t in_range = 0;	// Set once pH mixing gets into correct range
					uint8_t B2_cond_check = 1;
					// Prime B2 at beginning only once, don't push back until after done mixing
					while(priming_index < MAX_TIMES_TO_MIX && (gui32Error & ABORT_ERRORS) == 0 && B2_cond_check > 0)
					{
						// Prime a little B2 before test to clear out any contamination
						DEBUG_PRINT(UARTprintf("Priming B2... \n");)
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
						PumpVolume(FW, PumpVol_air_bubble, Speed_Metering, 1);
						userDelay(valve_delay_after_air, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);
						PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_Metering, 1);
#ifdef PRIME_BUFFERS_TEST
						if(priming_index == 0)	// Only do an extra large prime on the very first prime
						{
							// In order to test if a big prime hurts things set biggest prime for testing purposes
							DEBUG_PRINT(UARTprintf("Adding Max Prime... \n");)
										PumpVolume(FW, 200, Speed_Metering, 1);
						}
#endif
						userDelay(valve_delay, 1);

#ifdef CL_FILL_WITH_CLEAN
						DEBUG_PRINT(UARTprintf("Pumping Clean to fill channel!\n");)
						if(ISEs.Config == PH_CL_CART)
							RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);
						else
							RunValveToPossition_Bidirectional_AbortReady(V_CLEAN, VALVE_STEPS_PER_POSITION);
#else
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
#endif
						PumpVolume(FW, PumpVol_plug + PumpVol_Solution + PumpVol_Rinse, Speed_Fast, 1);
						FindPossitionZeroPump();
						userDelay(valve_delay, 1);

						float Conductivity_B2;
#ifndef COND_SOLUTION_STRUCT
						if(Sols->pH_EEP_Cal_2 < 9) // This is Cal 3, not Cal 2
						{
						    if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
						    	Conductivity_B2 = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_1, 0);
						    else
						    	Conductivity_B2 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_1, 0);
						}
						else
						{
						    if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
						    	Conductivity_B2 = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_2, 0);
						    else
						    	Conductivity_B2 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);
						}
#else
						Conductivity_B2 = MeasureConductivity(Sols, 0);
#endif

						float T_Samp_B2 = MeasureTemperature(1);
						Conductivity_B2 /= (1 + COND_TCOMP_B2_MIX * (T_Samp_B2 - 25));
						DEBUG_PRINT(UARTprintf("Conductivity of B2 mix: %d uS/cm * 1000\n", (int) (Conductivity_B2 * 1000));)

						if(Conductivity_B2 > 250 + Conductivity)
						{
							B2_cond_check = 0;
						}
						else
						{
							gui32Error |= B2_PRIME_COND_ERROR; // Update error
							update_Error();
						}

						ConnectMemory(1);
						MemoryWrite(Test_page, OFFSET_B2_PRIME_COND, 4, (uint8_t *) &Conductivity_B2);

						priming_index++;
					}

					// B2 is primed, put bubble back in tube
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve
					PumpVolume(BW, PumpVol_tube_bubble * 4, Speed_Metering, 1);
					userDelay(valve_delay_metering, 1);
					RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);		// Move valve
					PumpVolume(BW, PumpVol_tube_bubble, Speed_Metering, 1);
					userDelay(valve_delay_metering, 1);
				}
#endif



			if((gui32Error & (ABORT_ERRORS | CL_CLEANING_OUT_OF_RANGE)) == 0)
				if(MEASURE_FCL)
				{
					update_Status(STATUS_TEST, OPERATION_SAMPLE_B1);

					uint8_t priming_index = 0;
//					uint8_t in_range = 0;	// Set once pH mixing gets into correct range
					uint8_t cond_error = 1;

					while(priming_index < MAX_TIMES_TO_MIX && cond_error > 0)
					{
					    Sensor_in_rinse = 0;
						if((gui32Error & ABORT_ERRORS) == 0)
						{
							// Prime a little B1 before test to clear out any contamination
							DEBUG_PRINT(UARTprintf("Priming B1... \n");)
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
							PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 1);
							userDelay(valve_delay_after_air, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);
							PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_Metering, 1);
#ifdef PRIME_BUFFERS_TEST
							if(priming_index == 0)	// Only do an extra large prime on the very first prime
							{
								// In order to test if a big prime hurts things set biggest prime for testing purposes
								DEBUG_PRINT(UARTprintf("Adding Max Prime... \n");)
								PumpVolume(FW, 200, Speed_Metering, 1);
							}
#endif
							userDelay(valve_delay, 1);
						}

#ifdef CL_FILL_WITH_CLEAN
						DEBUG_PRINT(UARTprintf("Pumping Clean to fill channel!\n");)
						if(ISEs.Config == PH_CL_CART)
							RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);
						else
							RunValveToPossition_Bidirectional_AbortReady(V_CLEAN, VALVE_STEPS_PER_POSITION);
#else
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
#endif
						PumpVolume(FW, PumpVol_plug + PumpVol_Solution + PumpVol_Rinse, Speed_Fast, 1);

//						FindPossitionZeroPump();
						userDelay(valve_delay, 1);

						priming_index++;
						float Conductivity_B1;

#ifndef COND_SOLUTION_STRUCT
						if(Sols->pH_EEP_Cal_2 < 9) // This is Cal 3, not Cal 2
						{
						    if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
						        Conductivity_B1 = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_1, 0);
						    else
						        Conductivity_B1 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_1, 0);
						}
						else
						{
						    if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
						        Conductivity_B1 = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_2, 0);
						    else
						        Conductivity_B1 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);
						}
#else
						Conductivity_B1 = MeasureConductivity(Sols, 0);
#endif

						T_Samp_B1 = MeasureTemperature(1);
						Conductivity_B1 /= (1 + COND_TCOMP_B1_MIX * (T_Samp_B1 - 25));
						DEBUG_PRINT(UARTprintf("Temperature of mix: %d C * 1000\n", (int) (T_Samp_B1 * 1000));)
						DEBUG_PRINT(UARTprintf("Conductivity of B1 mix: %d uS/cm * 1000\n", (int) (Conductivity_B1 * 1000));)

						if(Conductivity_B1 > 1000 + Conductivity)
						{
						     cond_error = 0;
						}
						else
						{
							gui32Error |= B1_PRIME_COND_ERROR; // Update error
							update_Error();
						}

						ConnectMemory(1);
						MemoryWrite(Test_page, OFFSET_B1_PRIME_COND, 4, (uint8_t *) &Conductivity_B1);
					}
					PumpVolume(FW, PumpVol_sample_rinse - (PumpVol_plug + PumpVol_Solution + PumpVol_Rinse), Speed_Fast, 1);
					float T_Therm_B1 = ReadThermistor();
					MemoryWrite(Test_page, OFFSET_B1_THERM_TEMP, 4, (uint8_t *) &T_Therm_B1);

					//
					// FCl, B1 Mixing
					//
					DEBUG_PRINT(UARTprintf("Mixing %d uL of B1... \n", (int) PumpVol_Buffer);)
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air

					if(CL_MIX_IN_AIR)
					{
						DEBUG_PRINT(UARTprintf("Pumping large air plug so arrays and reference are uncovered during mixing\n");)
						PumpVolume(FW, PumpVol_air_bubble + PumpVol_Clean + 16.8, Speed_Fast, 1);
					}
					else
						PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 1);
					FindPossitionZeroPump();
					userDelay(valve_delay_after_air, 1);

					// Pump buffer and solution
					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
					PumpStepperRunStepSpeed_AbortReady(FW, Steps_PreB1, Speed_Metering);
					userDelay(valve_delay_metering, 1);
					RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
					PumpVolume(FW, PumpVol_Buffer, Speed_Metering, 1);
					userDelay(valve_delay_metering, 1);

					//						if(Steps_B1 <= 600)
					//						{
					//							RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
					//							PumpStepperRunStepSpeed_AbortReady(FW, Steps_B1, Speed_Metering);
					//							userDelay(valve_delay_metering, 1);
					//						}
					//						else
					//						{
					//							uint16_t Steps_to_go = Steps_B1;
					//							while(Steps_to_go > 600)
					//							{
					//								Steps_to_go -= 600;
					//								RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
					//								PumpStepperRunStepSpeed_AbortReady(FW, 600, Speed_Metering);
					//								userDelay(valve_delay_metering, 1);
					//								RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
					//								PumpStepperRunStepSpeed_AbortReady(FW, 400, Speed_Metering);
					//								userDelay(valve_delay_metering, 1);
					//							}
					//							RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
					//							PumpStepperRunStepSpeed_AbortReady(FW, Steps_to_go, Speed_Metering);
					//							userDelay(valve_delay_metering, 1);
					//						}

					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
					PumpVolume(FW, PumpVol_PostB1, Speed_Fast, 1);
					userDelay(valve_delay, 1);
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
					PumpVolume(FW, PumpVol_air_bubble /*+ PumpVol_tube_bubble*/, Speed_Fast, 1);
//					userDelay(valve_delay_after_air, 1);

//					RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
//					PumpVolume(BW, PumpVol_tube_bubble, Speed_Metering, 1);
//					userDelay(valve_delay_metering, 1);

					//						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to air
					//						PumpStepperRunStepSpeed_AbortReady(FW, Steps_follow_B1, Speed_placing);
					//						userDelay(valve_delay_metering, 1);

					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
					PumpStepperMix(BW, Steps_cycles, Speed_Fast, mix_cycles_Cl);
					//						for(i = 0; i < mix_cycles; i++)
					//						{
					//							PumpStepperRunStepSpeed_AbortReady(BW, Steps_cycles, Speed_mixing);
					//							PumpStepperRunStepSpeed_AbortReady(FW, Steps_cycles, Speed_mixing);
					//						}

					userDelay(diffusion_time_Cl, 1);

//					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to air
					PumpVolume(FW, PumpVol_center, Speed_Fast, 1);

					SleepValve();

					float Conductivity_B1;
					if((gui32Error & ABORT_ERRORS) == 0)
					{
						Cl_nA_FCl = ReadClnA(HIGH_RANGE, Amp_Voltage_Set, CL_TRACE_TIME);

						DEBUG_PRINT(UARTprintf("FCl raw: %d nA * 1000\n", (int) (Cl_nA_FCl * 1000));)

						T_Samp_B1 = MeasureTemperature(1);

						//								Cl_nA_FCl /= (.015 * T_Samp_B1 + .622);
						Cl_nA_FCl /= (.013 * T_Samp_B1 + .668);	// Updated 3/16/2020
						DEBUG_PRINT(UARTprintf("FCl raw normalized to 25C: %d nA * 1000\n", (int) ((Cl_nA_FCl) * 1000));)
						MemoryWrite(Test_page, OFFSET_RAW_CL_FCL, 4, (uint8_t *) &Cl_nA_FCl);

						//							Conductivity_B1 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0) / (1 + COND_TCOMP_B1_MIX * (T_Samp_B1 - 25));
#ifndef COND_SOLUTION_STRUCT
						if(Sols->pH_EEP_Cal_2 < 9) // This is Cal 3, not Cal 2
						{
							if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
								Conductivity_B1 = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_1, 0);
							else
								Conductivity_B1 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_1, 0);
						}
						else
						{
							if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
								Conductivity_B1 = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_2, 0);
							else
								Conductivity_B1 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);
						}
#else
						Conductivity_B1 = MeasureConductivity(Sols, 0);
#endif
						Conductivity_B1 /= (1 + COND_TCOMP_B1_MIX * (T_Samp_B1 - 25));
						DEBUG_PRINT(UARTprintf("Temperature of mix: %d C * 1000\n", (int) (T_Samp_B1 * 1000));)
						DEBUG_PRINT(UARTprintf("Conductivity of B1 mix: %d uS/cm * 1000\n", (int) (Conductivity_B1 * 1000));)

						MemoryWrite(Test_page, OFFSET_B1_MIX_COND, 4, (uint8_t *) &Conductivity_B1);
						MemoryWrite(Test_page, OFFSET_B1_MIX_TEMP, 4, (uint8_t *) &T_Samp_B1);

						if(Conductivity_B1 < 4000)
						{
							gui32Error |= FCL_MIX_OUT_OF_RANGE;	// Update error
							update_Error();
						}
						else
						{
							//
							// Cl Measurement
							//
//							float Cl_FCl_ppm = nanf("");//, Cl_MCl_NH3 = -1;
							float Cl_FCl_Int = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_FCL_INT, 4));
							float Cl_FCl_Slope = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_FCL_SLOPE, 4));
							float Cl_FCl_Int_High = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_FCL_INT_HIGH, 4));
							float Cl_FCl_Slope_High = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_FCL_SLOPE_HIGH, 4));
							float Cl_FCl_Midpoint = Cl_FCl_Slope * ((Cl_FCl_Int_High - Cl_FCl_Int) / (Cl_FCl_Slope - Cl_FCl_Slope_High)) + Cl_FCl_Int;

							if(Cl_FCl_Slope != Cl_FCl_Slope || Cl_FCl_Slope_High != Cl_FCl_Slope_High)
							{
								Cl_FCl_Int = CL_FCL_INT;
								Cl_FCl_Slope = CL_FCL_SLOPE;
								Cl_FCl_Int_High = CL_FCL_INT_HIGH;
								Cl_FCl_Slope_High = CL_FCL_SLOPE_HIGH;
								Cl_FCl_Midpoint = Cl_FCl_Slope * ((Cl_FCl_Int_High - Cl_FCl_Int) / (Cl_FCl_Slope - Cl_FCl_Slope_High)) + Cl_FCl_Int;
							}

							if(MEASURE_FCL && (gui32Error & (FCL_MIX_OUT_OF_RANGE | CL_CLEANING_OUT_OF_RANGE)) == 0)
							{
								if(Cl_nA_FCl > Cl_FCl_Midpoint)
									Cl_FCl_ppm = ((Cl_nA_FCl - Cl_FCl_Int) / Cl_FCl_Slope);//*((Steps_Sample_B1 + (float) Steps_B1) / Steps_Sample_B1); // ppm Cl2
								else
									Cl_FCl_ppm = ((Cl_nA_FCl - Cl_FCl_Int_High) / Cl_FCl_Slope_High);//*((Steps_Sample_B1 + (float) Steps_B1) / Steps_Sample_B1); // ppm Cl2

								if(Cl_FCl_ppm < 0)
									Cl_FCl_ppm = 0;

								MemoryWrite(Test_page, OFFSET_TEST_FREE_CL, 4, (uint8_t *) &Cl_FCl_ppm);
							}
						}

						update_Test(Test_Number);
					}

#ifdef READ_CL_MIX_PH
					if(READ_CL_MIX_PH)
					{
						float ISE_E_Samp_B1[10] = {0,0,0,0,0,0,0,0,0,0};
						float * pH_H2_E_Samp_B1 = &ISE_E_Samp_B1[ISEs.pH_H2.index];
						float * pH_Cr_E_Samp_B1 = &ISE_E_Samp_B1[ISEs.pH_Cr.index];
						float * NH4_E_Samp_B1 = &ISE_E_Samp_B1[ISEs.NH4.index];

						float ISE_Samp_B1[10] = {0,0,0,0,0,0,0,0,0,0};
						float * pH_H2_Samp_B1 = &ISE_Samp_B1[ISEs.pH_H2.index];
						float * pH_Cr_Samp_B1 = &ISE_Samp_B1[ISEs.pH_Cr.index];
						float * NH4_NH3_N_Free_B1 = &ISE_Samp_B1[ISEs.NH4.index];
						CollectISEmV(ISE_E_Samp_B1, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);

						float T_Samp_B1 = MeasureTemperature(1);

						if(ISEs.Config == PH_CL_CART)
							pH_TCor_Rinse = Calc_pH_TCor(Sols->pH_EEP_Clean, T_RS, 25, Sols->K_T_pH_Clean_Sq, Sols->K_T_pH_Clean_Ln);

						//
						// pH Measurement
						//
						for(i = 0; i < ISEs.pH_H2.size; i++)
						{
							float pH_H2_Slope_Samp_B1T = pH_H2_EEP_Slope[i] * (T_Samp_B1 + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
							pH_H2_Samp_B1[i] = pH_TCor_Rinse + ((pH_H2_E_Samp_B1[i] - pH_H2_E_Rinse[i]) / pH_H2_Slope_Samp_B1T); // pH of sample
						}

						for(i = 0; i < ISEs.pH_Cr.size; i++)
						{
							float pH_Cr_Slope_Samp_B1T = pH_Cr_EEP_Slope[i] * (T_Samp_B1 + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
							pH_Cr_Samp_B1[i] = pH_TCor_Rinse + ((pH_Cr_E_Samp_B1[i] - pH_Cr_E_Rinse[i]) / pH_Cr_Slope_Samp_B1T); // pH of sample
						}

						if(ISEs.NH4.size > 0)
						{
							//
							// NH4 Measurement
							//
							float IS_B1;
							if(Conductivity_B1 > 62)
								IS_B1 = 0.000016 * Conductivity_B1;
							else
								IS_B1 = 0.00001 * Conductivity_B1;

							float NH4_Alpha_B1 = pow(10, -pH_Cr_Samp_B1[T_Chosen_pH]) / (pow(10, -pH_Cr_Samp_B1[T_Chosen_pH]) + pow(10, -(0.09018 + 2729.92/T_RS)));

							float pNH4_Rinse;
							if(Sols->Ca_EEP_Rinse < 10)	// Values are p-values
								pNH4_Rinse = Sols->NH4_EEP_Rinse;
							else	// Values are concentration
								pNH4_Rinse = Calc_pNH4(Sols->NH4_EEP_Rinse, pH_TCor_Rinse, 0, T_RS, Sols->IS_RINSE);

							// TODO: Enter potassium and sodium interference values
							float K_interference = 1.5; // ppm
							float Na_interference = 15; // ppm

							//							float NH4_NH3_N_Free_B1[2];
							for(i = 0; i < ISEs.NH4.size; i++)
							{
								float NH4_Slope_RST = NH4_EEP_Slope[i] * (T_RS + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
								float NH4_Samp = pNH4_Rinse + ((NH4_E_Samp_B1[i] - NH4_E_Rinse[i]) / NH4_Slope_RST);	// pNH4

								float Activity_NH4_K_Na = pow(10, -NH4_Samp);

								float Activity_K = K_interference / 39098.3 * Lambda_K(T_RS, IS_B1);
								float Activity_Na = Na_interference / 22989.8 * Lambda_Na(T_RS, IS_B1);
								float Activity_Total = Activity_NH4_K_Na + ((1 - pow(10, LOG_K_K_NH4)) * Activity_K) + ((1 - pow(10, LOG_K_NA_NH4)) * Activity_Na);
								float Activity_NH4 = Activity_Total - Activity_K - Activity_Na;

								float NH4_Ammonium_B1 = Activity_NH4 / Lambda_NH4(T_RS, IS_B1) * 14000;
								NH4_NH3_N_Free_B1[i] = NH4_Ammonium_B1 / NH4_Alpha_B1;
							}
						}

						DEBUG_PRINT(UARTprintf("pH of mixed B1:\n");)
						DEBUG_PRINT(UARTprintf("pH H2:");)
						for(i = 0; i < ISEs.pH_H2.size; i++)
							{DEBUG_PRINT(UARTprintf("\t%d", (int) (pH_H2_Samp_B1[i] * 1000));)}
						DEBUG_PRINT(UARTprintf("\n");)
						DEBUG_PRINT(UARTprintf("pH Cr:");)
						for(i = 0; i < ISEs.pH_Cr.size; i++)
							{DEBUG_PRINT(UARTprintf("\t%d", (int) (pH_Cr_Samp_B1[i] * 1000));)}
						DEBUG_PRINT(UARTprintf("\n");)
						if(ISEs.NH4.size > 0)
							{DEBUG_PRINT(UARTprintf("NH4 of B1 Mix:\t=%d/1000\t=%d/1000\n\n", (int) (NH4_NH3_N_Free_B1[0] * 1000), (int) (NH4_NH3_N_Free_B1[1] * 1000));)}

						DEBUG_PRINT(UARTprintf("Temp Cal, B1 Mix: %d, %d\n", (int) (T_EEP_Cal * 1000), (int) (T_Samp_B1 * 1000));)
						DEBUG_PRINT(UARTprintf("pH TCor Rinse: %d\n", (int) (pH_TCor_Rinse * 1000));)

						// Save pH values, takes over the Alkalinity pH spots
						MemoryWrite(Test_page, OFFSET_TEST_PH_1_T1_1, 4, (uint8_t *) &ISE_Samp_B1[0]);
						MemoryWrite(Test_page, OFFSET_TEST_PH_2_T1_1, 4, (uint8_t *) &ISE_Samp_B1[1]);
						MemoryWrite(Test_page, OFFSET_TEST_PH_3_T1_1, 4, (uint8_t *) &ISE_Samp_B1[2]);

						// Write the rest of the values to the memory
						for(i = 0; i < 7; i++)
						{
							MemoryWrite(Test_page, OFFSET_ISE_4_T1_1 + (i * 4), 4, (uint8_t *) &ISE_Samp_B1[3 + i]);
						}
					}
#endif

					// Leave RE and CE floating
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);		// RE floating
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);		// CE floating

					RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
					PumpVolume(BW, PumpVol_tube_bubble, Speed_Metering, 1);
					userDelay(valve_delay_metering, 1);

					//						// Push B1 back in tube here so it is before B2 and C2 priming but after read so we take less time
					//						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
					//						PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble + Steps_tube_bubble, Speed_placing);
					//						userDelay(valve_delay_after_air, 1);
					//						RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to air
					//						PumpStepperRunStepSpeed_AbortReady(BW, Steps_tube_bubble, Speed_placing);
					//						userDelay(valve_delay, 1);

				}

#ifdef TESTING_MODE
			uint64_t free_clock = TimerValueGet64(WTIMER0_BASE);
			DEBUG_PRINT(UARTprintf("Time to free: %d\n", (uint32_t) ((free_clock - clean_clock) / SysCtlClockGet()));)
			DEBUG_PRINT(UARTprintf("Total time elapsed: %d s, or %d m %d s\n", (uint32_t) ((free_clock - start_clock)/SysCtlClockGet()), (uint32_t) ((free_clock - start_clock)/SysCtlClockGet())/60,(uint32_t) ((free_clock - start_clock)/SysCtlClockGet())%60);)
#endif

			//
			// TCL, B2
			//
			PrintTime();
			if((gui32Error & (ABORT_ERRORS | CL_CLEANING_OUT_OF_RANGE)) == 0)
				if(MEASURE_TCL)
				{
					update_Status(STATUS_TEST, OPERATION_SAMPLE_B2);

					uint8_t priming_index = 0;
//					uint8_t in_range = 0;	// Set once pH mixing gets into correct range
					uint8_t c2_cond_check = 1;
					// Prime B2 at beginning only once, don't push back until after done mixing
					while(priming_index < MAX_TIMES_TO_MIX && (gui32Error & ABORT_ERRORS) == 0 && c2_cond_check > 0)
					{
						// Prime a little C2 before test to clear out any contamination
						DEBUG_PRINT(UARTprintf("Priming C2... \n");)
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
						PumpVolume(FW, PumpVol_air_bubble, Speed_Metering, 1);
						userDelay(valve_delay_after_air, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_C2, VALVE_STEPS_PER_POSITION);
						PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_Metering, 1);
#ifdef PRIME_BUFFERS_TEST
						if(priming_index == 0)	// Only do an extra large prime on the very first prime
						{
							// In order to test if a big prime hurts things set biggest prime for testing purposes
							DEBUG_PRINT(UARTprintf("Adding Max Prime... \n");)
										PumpVolume(FW, 200, Speed_Metering, 1);
						}
#endif
						userDelay(valve_delay, 1);

#ifdef CL_FILL_WITH_CLEAN
						DEBUG_PRINT(UARTprintf("Pumping Clean to fill channel!\n");)
						if(ISEs.Config == PH_CL_CART)
							RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);
						else
							RunValveToPossition_Bidirectional_AbortReady(V_CLEAN, VALVE_STEPS_PER_POSITION);
#else
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
#endif
						PumpVolume(FW, PumpVol_plug + PumpVol_Solution + PumpVol_Rinse, Speed_Fast, 1);
						FindPossitionZeroPump();
						userDelay(valve_delay, 1);

						float Conductivity_C2;
#ifndef COND_SOLUTION_STRUCT
						if(Sols->pH_EEP_Cal_2 < 9) // This is Cal 3, not Cal 2
						{
						    if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
						    	Conductivity_C2 = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_1, 0);
						    else
						    	Conductivity_C2 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_1, 0);
						}
						else
						{
						    if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
						    	Conductivity_C2 = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_2, 0);
						    else
						    	Conductivity_C2 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);
						}
#else
						Conductivity_C2 = MeasureConductivity(Sols, 0);
#endif

						float T_Samp_C2 = MeasureTemperature(1);
						Conductivity_C2 /= (1 + COND_TCOMP_B2_MIX * (T_Samp_C2 - 25));
						DEBUG_PRINT(UARTprintf("Conductivity of C2 mix: %d uS/cm * 1000\n", (int) (Conductivity_C2 * 1000));)

						if(Conductivity_C2 > 250 + Conductivity)
						{
							c2_cond_check = 0;
						}
						else
						{
							gui32Error |= C2_PRIME_COND_ERROR; // Update error
							update_Error();
						}

						ConnectMemory(1);
						MemoryWrite(Test_page, OFFSET_C2_PRIME_COND, 4, (uint8_t *) &Conductivity_C2);

						priming_index++;
					}
#ifdef CL_FILL_WITH_CLEAN
					DEBUG_PRINT(UARTprintf("Pumping Clean to fill channel!\n");)
					if(ISEs.Config == PH_CL_CART)
						RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);
					else
						RunValveToPossition_Bidirectional_AbortReady(V_CLEAN, VALVE_STEPS_PER_POSITION);
#else
					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
#endif
					PumpVolume(FW, PumpVol_sample_rinse - (PumpVol_plug + PumpVol_Solution + PumpVol_Rinse), Speed_Fast, 1);
					float T_Therm_B2 = ReadThermistor();
					MemoryWrite(Test_page, OFFSET_B2_THERM_TEMP, 4, (uint8_t *) &T_Therm_B2);
					userDelay(valve_delay_after_air, 1);

					// Pump and mix conditioner with sample
					DEBUG_PRINT(UARTprintf("Mixing %d uL of C2... \n", (int) PumpVol_C2);)

					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air

					if(CL_MIX_IN_AIR)
					{
						DEBUG_PRINT(UARTprintf("Pumping large air plug so arrays and reference are uncovered during mixing\n");)
						PumpVolume(FW, PumpVol_air_bubble + PumpVol_air_bubble + PumpVol_Clean, Speed_Fast, 1);
					}
					else
						PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 1);
					FindPossitionZeroPump();
					userDelay(valve_delay_after_air, 1);

					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);	// Move valve to sample
					PumpStepperRunStepSpeed_AbortReady(FW, Steps_PreB1, Speed_Metering);
					userDelay(valve_delay_metering, 1);
					RunValveToPossition_Bidirectional_AbortReady(V_C2, VALVE_STEPS_PER_POSITION);		// Move valve to C2
					PumpVolume(FW, PumpVol_C2, Speed_Metering, 1);
					userDelay(valve_delay_metering, 1);
					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);	// Move valve to sample
					PumpVolume(FW, PumpVol_PostB1, Speed_Metering, 1);
					userDelay(valve_delay_metering, 1);
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air

					PumpVolume(FW, PumpVol_air_bubble + PumpVol_tube_bubble, Speed_Metering, 1);
					userDelay(valve_delay_after_air, 1);

					// Push C2 back a little in valve to help avoid it contaminating B1 and causing us to lose FCl
					RunValveToPossition_Bidirectional_AbortReady(V_C2, VALVE_STEPS_PER_POSITION);		// Move valve to air
					PumpVolume(BW, PumpVol_tube_bubble, Speed_Metering, 1);
					userDelay(valve_delay, 1);

					//						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);	// Move valve to sample
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
					PumpVolume(FW, PumpVol_follow_C2, Speed_Fast, 1);
					//						userDelay(valve_delay_metering, 1);

					PumpStepperMix(BW, Steps_cycles, Speed_Fast, mix_cycles_Cl);

					userDelay(diffusion_time_Cl, 1);	// Delay to let diffusion happen

#ifdef MEASURE_C2_MIX
					if(MEASURE_C2_MIX)
					{
						PumpVolume(FW, PumpVol_center - PumpVol_follow_C2, Speed_Fast, 1);

						float ISE_E_Samp_B2[10] = {0,0,0,0,0,0,0,0,0,0};
						float * pH_H2_E_Samp_B2 = &ISE_E_Samp_B2[ISEs.pH_H2.index];
						float * pH_Cr_E_Samp_B2 = &ISE_E_Samp_B2[ISEs.pH_Cr.index];
						float * NH4_E_Samp_B2 = &ISE_E_Samp_B2[ISEs.NH4.index];

						float ISE_Samp_B2[10] = {0,0,0,0,0,0,0,0,0,0};
						float * pH_H2_Samp_B2 = &ISE_Samp_B2[ISEs.pH_H2.index];
						float * pH_Cr_Samp_B2 = &ISE_Samp_B2[ISEs.pH_Cr.index];
						float * NH4_NH3_N_Free_B2 = &ISE_Samp_B2[ISEs.NH4.index];
						CollectISEmV(ISE_E_Samp_B2, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);

						float T_Samp_B2 = MeasureTemperature(1);

						//
						// pH Measurement
						//
						for(i = 0; i < ISEs.pH_H2.size; i++)
						{
							float pH_H2_Slope_Samp_B2T = pH_H2_EEP_Slope[i] * (T_Samp_B2 + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
							pH_H2_Samp_B2[i] = pH_TCor_Rinse + ((pH_H2_E_Samp_B2[i] - pH_H2_E_Rinse[i]) / pH_H2_Slope_Samp_B2T); // pH of sample
						}

						for(i = 0; i < ISEs.pH_Cr.size; i++)
						{
							float pH_Cr_Slope_Samp_B2T = pH_Cr_EEP_Slope[i] * (T_Samp_B2 + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
							pH_Cr_Samp_B2[i] = pH_TCor_Rinse + ((pH_Cr_E_Samp_B2[i] - pH_Cr_E_Rinse[i]) / pH_Cr_Slope_Samp_B2T); // pH of sample
						}

						if(ISEs.NH4.size > 0)
						{
							float Conductivity_C2 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0) / (1 + COND_TCOMP_B2_MIX * (T_Samp_B2 - 25));
							DEBUG_PRINT(UARTprintf("Temperature of mix: %d C * 1000\n", (int) (T_Samp_B2 * 1000));)
							DEBUG_PRINT(UARTprintf("Conductivity of C2 mix: %d uS/cm * 1000\n", (int) (Conductivity_C2 * 1000));)

							//
							// NH4 Measurement
							//
							float IS_B2;
							if(Conductivity_C2 > 62)
								IS_B2 = 0.000016 * Conductivity_C2;
							else
								IS_B2 = 0.00001 * Conductivity_C2;

							float NH4_Alpha_B2 = pow(10, -pH_Cr_Samp_B2[T_Chosen_pH]) / (pow(10, -pH_Cr_Samp_B2[T_Chosen_pH]) + pow(10, -(0.09018 + 2729.92/T_RS)));

							float pNH4_Rinse;
							if(Sols->Ca_EEP_Rinse < 10)	// Values are p-values
								pNH4_Rinse = Sols->NH4_EEP_Rinse;
							else	// Values are concentration
								pNH4_Rinse = Calc_pNH4(Sols->NH4_EEP_Rinse, pH_TCor_Rinse, 0, T_RS, Sols->IS_RINSE);

							// TODO: Enter potassium and sodium interference values
							float K_interference = 1.5; // ppm
							float Na_interference = 15; // ppm

							//							float NH4_NH3_N_Free_B2[2];
							for(i = 0; i < ISEs.NH4.size; i++)
							{
								float NH4_Slope_RST = NH4_EEP_Slope[i] * (T_RS + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
								float NH4_Samp = pNH4_Rinse + ((NH4_E_Samp_B2[i] - NH4_E_Rinse[i]) / NH4_Slope_RST);	// pNH4

								float Activity_NH4_K_Na = pow(10, -NH4_Samp);

								float Activity_K = K_interference / 39098.3 * Lambda_K(T_RS, IS_B2);
								float Activity_Na = Na_interference / 22989.8 * Lambda_Na(T_RS, IS_B2);
								float Activity_Total = Activity_NH4_K_Na + ((1 - pow(10, LOG_K_K_NH4)) * Activity_K) + ((1 - pow(10, LOG_K_NA_NH4)) * Activity_Na);
								float Activity_NH4 = Activity_Total - Activity_K - Activity_Na;

								float NH4_Ammonium_B2 = Activity_NH4 / Lambda_NH4(T_RS, IS_B2) * 14000;
								NH4_NH3_N_Free_B2[i] = NH4_Ammonium_B2 / NH4_Alpha_B2;
							}
						}

						DEBUG_PRINT(UARTprintf("pH of mixed C2:\n");)
						DEBUG_PRINT(UARTprintf("pH H2:");)
						for(i = 0; i < ISEs.pH_H2.size; i++)
							{DEBUG_PRINT(UARTprintf("\t%d", (int) (pH_H2_Samp_B2[i] * 1000));)}
						DEBUG_PRINT(UARTprintf("\n");)
						DEBUG_PRINT(UARTprintf("pH Cr:");)
						for(i = 0; i < ISEs.pH_Cr.size; i++)
							{DEBUG_PRINT(UARTprintf("\t%d", (int) (pH_Cr_Samp_B2[i] * 1000));)}
						DEBUG_PRINT(UARTprintf("\n");)
						if(ISEs.NH4.size > 0)
							{DEBUG_PRINT(UARTprintf("NH4 of C2 Mix: =%d/1000\t=%d/1000\n\n", (int) (NH4_NH3_N_Free_B2[0] * 1000), (int) (NH4_NH3_N_Free_B2[1] * 1000));)}

						PumpVolume(BW, PumpVol_center - PumpVol_follow_C2, Speed_Fast, 1);
					}
#endif

					// Prime a little B2 before test to clear out any contamination
					DEBUG_PRINT(UARTprintf("Priming B2... \n");)
					//						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
					//						PumpVolume(FW, PumpVol_air_bubble, Speed_Metering, 1);
					//						userDelay(valve_delay_after_air, 1);
					RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);
					PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_Metering, 1);
					userDelay(valve_delay, 1);
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
					PumpVolume(FW, PumpVol_air_bubble, Speed_Metering, 1);
					userDelay(valve_delay_after_air, 1);

					// Pump mixed conditioner/solution back and mix with buffer
					DEBUG_PRINT(UARTprintf("Mixing %d uL of B2... \n", (int) PumpVol_Buffer);)

					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
					PumpVolume(BW, PumpVol_back + PumpVol_tube_prime_buffers + PumpVol_air_bubble, Speed_Metering, 1);
					userDelay(valve_delay, 1);
					int32_t Steps_to_align_pump = 1000 - (g_PumpStepsTravelled % 1000);
					if(Steps_to_align_pump == 1000)
						Steps_to_align_pump = 0;
					float Volume_to_align_pump = Steps_to_align_pump * PumpVolRev / Pump_Ratio / 1000;
					// Need to remove the dead spot volume, luckily with the PumpVolume code we will never stop the pump in the dead spot so it'll be an all or nothing calculation
					if(Steps_to_align_pump > 250)	// Check if we will be passing through dead spot, since we can't be inside the dead spot can merely check against center of dead spot
						Volume_to_align_pump -= (1 - Pump_Ratio) * PumpVolRev / Pump_Ratio;	// Remove the volume of the dead spot
					PumpStepperRunStepSpeed_AbortReady(FW, 1000 + Steps_to_align_pump, Speed_Metering);	// Buffer seemed to meter better if the pump was last pumping forward
					userDelay(valve_delay_metering, 1);

					//						if(Steps_B2 <= 600)
					//						{
					RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
					PumpVolume(FW, PumpVol_Buffer, Speed_Metering, 1);
					userDelay(valve_delay_metering, 1);
					//						}
					//						else
					//						{
					//							uint16_t Steps_to_go = Steps_B2;
					//							while(Steps_to_go > 600)
					//							{
					//								Steps_to_go -= 600;
					//								RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
					//								PumpStepperRunStepSpeed_AbortReady(FW, 600, Speed_Metering);
					//								userDelay(valve_delay_metering, 1);
					//								RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
					//								PumpStepperRunStepSpeed_AbortReady(FW, 400, Speed_Metering);
					//								userDelay(valve_delay_metering, 1);
					//							}
					//							RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
					//							PumpStepperRunStepSpeed_AbortReady(FW, Steps_to_go, Speed_Metering);
					//							userDelay(valve_delay_metering, 1);
					//						}

					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
					PumpVolume(FW, PumpVol_forward - Volume_to_align_pump, Speed_Metering, 1);
					userDelay(valve_delay_metering, 1);

					PumpStepperMix(BW, Steps_cycles, Speed_Fast, mix_cycles_Cl);

					userDelay(diffusion_time_Cl, 1);	// Delay to let diffusion happen

					PumpVolume(FW, PumpVol_center_B2, Speed_Fast, 1);

					SleepValve();

					float Conductivity_B2;
					if((gui32Error & ABORT_ERRORS) == 0)
					{
						//							DEBUG_PRINT(UARTprintf("pH is in correct range, running chlorine test!\n");)
						T_Samp_B2 = MeasureTemperature(1);
						MemoryWrite(Test_page, OFFSET_B2_MIX_START_TEMP, 4, (uint8_t *) &T_Samp_B2);

						Cl_nA_TCl = ReadClnA(HIGH_RANGE, Amp_Voltage_Set, CL_TRACE_TIME);

						DEBUG_PRINT(UARTprintf("TCl raw: %d nA * 1000\n", (int) (Cl_nA_TCl * 1000));)

						T_Samp_B2 = MeasureTemperature(1);

						Cl_nA_TCl /= 0.014 * T_Samp_B2 + 0.654;
						DEBUG_PRINT(UARTprintf("TCl raw normalized to 25C: %d nA * 1000\n", (int) (Cl_nA_TCl * 1000));)
						MemoryWrite(Test_page, OFFSET_RAW_CL_TCL, 4, (uint8_t *) &Cl_nA_TCl);

						//							Conductivity_B2 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0) / (1 + COND_TCOMP_B2_MIX * (T_Samp_B2 - 25));
#ifndef COND_SOLUTION_STRUCT
						if(Sols->pH_EEP_Cal_2 < 9) // This is Cal 3, not Cal 2
						{
							if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
								Conductivity_B2 = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_1, 0);
							else
								Conductivity_B2 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_1, 0);
						}
						else
						{
							if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
								Conductivity_B2 = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_2, 0);
							else
								Conductivity_B2 = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);
						}
#else
						Conductivity_B2 = MeasureConductivity(Sols, 0);
#endif
						Conductivity_B2 /= (1 + COND_TCOMP_B2_MIX * (T_Samp_B2 - 25));

						DEBUG_PRINT(UARTprintf("Temperature of mix: %d C * 1000\n", (int) (T_Samp_B2 * 1000));)
						DEBUG_PRINT(UARTprintf("Conductivity of B2 mix: %d uS/cm * 1000\n", (int) (Conductivity_B2 * 1000));)

						MemoryWrite(Test_page, OFFSET_B2_MIX_COND, 4, (uint8_t *) &Conductivity_B2);
						MemoryWrite(Test_page, OFFSET_B2_MIX_TEMP, 4, (uint8_t *) &T_Samp_B2);

						if(Conductivity_B2 < 9000)
						{
							gui32Error |= TCL_MIX_OUT_OF_RANGE;	// Update error
							update_Error();
						}
						else
						{
							//
							// Cl Measurement
							//
							float Cl_TCl_Slope = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_TCL_SLOPE, 4));
							float Cl_TCl_Int = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_TCL_INT, 4));
							float Cl_TCl_Slope_High = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_TCL_SLOPE_HIGH, 4));
							float Cl_TCl_Int_High = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_TCL_INT_HIGH, 4));
							float Cl_TCl_Midpoint = Cl_TCl_Slope * ((Cl_TCl_Int_High - Cl_TCl_Int) / (Cl_TCl_Slope - Cl_TCl_Slope_High)) + Cl_TCl_Int;

							if(Cl_TCl_Slope_High != Cl_TCl_Slope_High || Cl_TCl_Slope != Cl_TCl_Slope)
							{
								Cl_TCl_Slope = CL_TCL_SLOPE;
								Cl_TCl_Int = CL_TCL_INT;
								Cl_TCl_Slope_High = CL_TCL_SLOPE_HIGH;
								Cl_TCl_Int_High = CL_TCL_INT_HIGH;

								Cl_TCl_Midpoint = Cl_TCl_Slope * ((Cl_TCl_Int_High - Cl_TCl_Int) / (Cl_TCl_Slope - Cl_TCl_Slope_High)) + Cl_TCl_Int;
							}

							if(MEASURE_TCL && (gui32Error & (TCL_MIX_OUT_OF_RANGE | CL_CLEANING_OUT_OF_RANGE)) == 0)
							{
								// 2/26/2020: Removed dilution math because dilution during calibration is same as dilution during tests
								if(Cl_nA_TCl > Cl_TCl_Midpoint)
									Cl_TCl_ppm = ((Cl_nA_TCl - Cl_TCl_Int) / Cl_TCl_Slope);//*((Steps_Sample_B2 + (float) Steps_B2 + (float) Steps_C2) / Steps_Sample_B2); // ppm Cl2
								else
									Cl_TCl_ppm = ((Cl_nA_TCl - Cl_TCl_Int_High) / Cl_TCl_Slope_High);//*((Steps_Sample_B2 + (float) Steps_B2 + (float) Steps_C2) / Steps_Sample_B2); // ppm Cl2

								if(Cl_TCl_ppm < 0)
									Cl_TCl_ppm = 0;

								MemoryWrite(Test_page, OFFSET_TEST_TOTAL_CL, 4, (uint8_t *) &Cl_TCl_ppm);

								if(Cl_FCl_ppm == Cl_FCl_ppm && Cl_TCl_ppm == Cl_TCl_ppm)
								{
									Cl_MCl_ppm = Cl_TCl_ppm - Cl_FCl_ppm; 				// ppm Cl2

									if(Cl_MCl_ppm < 0)
										Cl_MCl_ppm = 0;

									MemoryWrite(Test_page, OFFSET_TEST_MONO, 4, (uint8_t *) &Cl_MCl_ppm);
								}

								if(MEASURE_FCL && MEASURE_TCL && ISEs.NH4.size > 0  && (gui32Error & (FCL_MIX_OUT_OF_RANGE | TCL_MIX_OUT_OF_RANGE | CL_CLEANING_OUT_OF_RANGE)) == 0)// && ((gui32Error & (TCL_MIX_OUT_OF_RANGE | FCL_MIX_OUT_OF_RANGE)) == 0))
								{


									if(Cond_Cal_Status && ISE_Cal_Status[ISEs.NH4.index + T_Chosen_NH4] && ISE_Cal_Status[ISEs.pH_Cr.index + T_Chosen_pH] && ISEs.NH4.size > 0)
									{
										NH4_TNH3 = NH4_NH3_N_Free[T_Chosen_NH4] + Cl_MCl_ppm * 0.1986;	// Calculate Total Ammonia
										if(NH4_TNH3 == 0)
											NH4_Cl_NH3 = 0;
										else
											NH4_Cl_NH3 = Cl_TCl_ppm / NH4_TNH3;

										Nitrification_Capacity = (0.1986 * 11 * (Cl_TCl_ppm - Cl_FCl_ppm) / 10) + NH4_NH3_N_Free[T_Chosen_NH4];

										MemoryWrite(Test_page, OFFSET_TEST_TOTAL_NH4_MONO, 4, (uint8_t *) &NH4_TNH3);
										MemoryWrite(Test_page, OFFSET_TEST_CL_NH4_RATIO, 4, (uint8_t *) &NH4_Cl_NH3);
										MemoryWrite(Test_page, OFFSET_TEST_BFR, 4, (uint8_t *) &Nitrification_Capacity);
									}
								}
							}
						}

						update_Test(Test_Number);
					}	// gui32Error abort errors

#ifdef READ_CL_MIX_PH
					if(READ_CL_MIX_PH)
					{
						float ISE_E_Samp_B2[10] = {0,0,0,0,0,0,0,0,0,0};
						float * pH_H2_E_Samp_B2 = &ISE_E_Samp_B2[ISEs.pH_H2.index];
						float * pH_Cr_E_Samp_B2 = &ISE_E_Samp_B2[ISEs.pH_Cr.index];
						float * NH4_E_Samp_B2 = &ISE_E_Samp_B2[ISEs.NH4.index];

						float ISE_Samp_B2[10] = {0,0,0,0,0,0,0,0,0,0};
						float * pH_H2_Samp_B2 = &ISE_Samp_B2[ISEs.pH_H2.index];
						float * pH_Cr_Samp_B2 = &ISE_Samp_B2[ISEs.pH_Cr.index];
						float * NH4_NH3_N_Free_B2 = &ISE_Samp_B2[ISEs.NH4.index];
						CollectISEmV(ISE_E_Samp_B2, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);

						float T_Samp_B2 = MeasureTemperature(1);

						//
						// pH Measurement
						//
						for(i = 0; i < ISEs.pH_H2.size; i++)
						{
							float pH_H2_Slope_Samp_B2T = pH_H2_EEP_Slope[i] * (T_Samp_B2 + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
							pH_H2_Samp_B2[i] = pH_TCor_Rinse + ((pH_H2_E_Samp_B2[i] - pH_H2_E_Rinse[i]) / pH_H2_Slope_Samp_B2T); // pH of sample
						}

						for(i = 0; i < ISEs.pH_Cr.size; i++)
						{
							float pH_Cr_Slope_Samp_B2T = pH_Cr_EEP_Slope[i] * (T_Samp_B2 + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
							pH_Cr_Samp_B2[i] = pH_TCor_Rinse + ((pH_Cr_E_Samp_B2[i] - pH_Cr_E_Rinse[i]) / pH_Cr_Slope_Samp_B2T); // pH of sample
						}

						if(ISEs.NH4.size > 0)
						{
							//
							// NH4 Measurement
							//
							float IS_B2;
							if(Conductivity_B2 > 62)
								IS_B2 = 0.000016 * Conductivity_B2;
							else
								IS_B2 = 0.00001 * Conductivity_B2;

							float NH4_Alpha_B2 = pow(10, -pH_Cr_Samp_B2[T_Chosen_pH]) / (pow(10, -pH_Cr_Samp_B2[T_Chosen_pH]) + pow(10, -(0.09018 + 2729.92/T_RS)));

							float pNH4_Rinse;
							if(Sols->Ca_EEP_Rinse < 10)	// Values are p-values
								pNH4_Rinse = Sols->NH4_EEP_Rinse;
							else	// Values are concentration
								pNH4_Rinse = Calc_pNH4(Sols->NH4_EEP_Rinse, pH_TCor_Rinse, 0, T_RS, Sols->IS_RINSE);

							// TODO: Enter potassium and sodium interference values
							float K_interference = 1.5; // ppm
							float Na_interference = 15; // ppm

							//							float NH4_NH3_N_Free_B2[2];
							for(i = 0; i < ISEs.NH4.size; i++)
							{
								float NH4_Slope_RST = NH4_EEP_Slope[i] * (T_RS + 273) / (T_EEP_Cal + 273);	// Temperature corrected slope
								float NH4_Samp = pNH4_Rinse + ((NH4_E_Samp_B2[i] - NH4_E_Rinse[i]) / NH4_Slope_RST);	// pNH4

								float Activity_NH4_K_Na = pow(10, -NH4_Samp);

								float Activity_K = K_interference / 39098.3 * Lambda_K(T_RS, IS_B2);
								float Activity_Na = Na_interference / 22989.8 * Lambda_Na(T_RS, IS_B2);
								float Activity_Total = Activity_NH4_K_Na + ((1 - pow(10, LOG_K_K_NH4)) * Activity_K) + ((1 - pow(10, LOG_K_NA_NH4)) * Activity_Na);
								float Activity_NH4 = Activity_Total - Activity_K - Activity_Na;

								float NH4_Ammonium_B2 = Activity_NH4 / Lambda_NH4(T_RS, IS_B2) * 14000;
								NH4_NH3_N_Free_B2[i] = NH4_Ammonium_B2 / NH4_Alpha_B2;
							}
						}

						DEBUG_PRINT(UARTprintf("pH of mixed B2:\n");)
						DEBUG_PRINT(UARTprintf("pH H2:");)
						for(i = 0; i < ISEs.pH_H2.size; i++)
							{DEBUG_PRINT(UARTprintf("\t%d", (int) (pH_H2_Samp_B2[i] * 1000));)}
						DEBUG_PRINT(UARTprintf("\n");)
						DEBUG_PRINT(UARTprintf("pH Cr:");)
						for(i = 0; i < ISEs.pH_Cr.size; i++)
							{DEBUG_PRINT(UARTprintf("\t%d", (int) (pH_Cr_Samp_B2[i] * 1000));)}
						DEBUG_PRINT(UARTprintf("\n");)
						if(ISEs.NH4.size > 0)
							{DEBUG_PRINT(UARTprintf("NH4 of B2 Mix: =%d/1000\t=%d/1000\n\n", (int) (NH4_NH3_N_Free_B2[0] * 1000), (int) (NH4_NH3_N_Free_B2[1] * 1000));)}

						// Save pH values, takes over the Alkalinity pH spots
						MemoryWrite(Test_page, OFFSET_TEST_PH_1_T1_2, 4, (uint8_t *) &ISE_Samp_B2[0]);
						MemoryWrite(Test_page, OFFSET_TEST_PH_1_T1_2, 4, (uint8_t *) &ISE_Samp_B2[1]);
						MemoryWrite(Test_page, OFFSET_TEST_PH_1_T1_2, 4, (uint8_t *) &ISE_Samp_B2[2]);

						// Write the rest of the values to the memory
						for(i = 0; i < 7; i++)
						{
							MemoryWrite(Test_page, OFFSET_ISE_4_T1_2 + (i * 4), 4, (uint8_t *) &ISE_Samp_B2[3 + i]);
						}
					}
#endif

					// RE and CE floating
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);		// Leave RE floating
					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);		// Leave CE floating


					// After running TCl push B2 buffer back slightly into pouch
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
					PumpVolume(FW, PumpVol_air_bubble + PumpVol_tube_bubble, Speed_Metering, 1);
					userDelay(valve_delay_after_air, 1);
					RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);		// Move valve to air
					PumpVolume(BW, PumpVol_tube_bubble, Speed_Metering, 1);
					userDelay(valve_delay, 1);
				}

#ifdef TESTING_MODE
			uint64_t total_clock = TimerValueGet64(WTIMER0_BASE);
			DEBUG_PRINT(UARTprintf("Time to total: %d\n", (uint32_t) ((total_clock - free_clock) / SysCtlClockGet()));)
			DEBUG_PRINT(UARTprintf("Total time elapsed: %d s, or %d m %d s\n", (uint32_t) ((total_clock - start_clock)/SysCtlClockGet()), (uint32_t) ((total_clock - start_clock)/SysCtlClockGet())/60,(uint32_t) ((total_clock - start_clock)/SysCtlClockGet())%60);)
#endif


#ifdef MEASURE_NITRITE
			// Create variables for Nitrite
			float Nitrite_Samp_nA = 0;
			float Nitrite_SampNitrite_nA = 0;
			float T_Nitrite_Samp = T_assume;
			float T_Nitrite_SampNitrite = T_assume;
			float Nitrite_Amp_Voltage_Set = 1031 - Ref_drift;
			if(REF_DRIFT != 0 && SATURATED_KCL_REF != 0)
				Nitrite_Amp_Voltage_Set = 1031 - Calculate_Ref_Drift(SATURATED_KCL_REF, MeasureTemperature(1));
			//			float T_Blank_1 = T_assume;
			//			float T_Blank_2 = T_assume;
			//			float Nitrite_Blank_1 = 0;
			//			float Nitrite_Blank_2 = 0;
			if(MEASURE_NITRITE && ISEs.Config == PH_CL_CART)
			{
				//
				// Clean a second time before measuring the nitrite
				//
				PrintTime();
				if((gui32Error & ABORT_ERRORS) == 0)
					if(CLEAN_AMPS_TEST && MEASURE_NITRITE && ISEs.Config == PH_CL_CART)
					{
						if(MEASURE_FCL || MEASURE_TCL)
						{
							// Pump in rinse over amperometrics for cleaning
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
							FindPossitionZeroPump();
							userDelay(valve_delay_after_air, 1);
							DEBUG_PRINT(UARTprintf("Pumping cleaning solution to clean amperometrics!\n");)
							RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);

							if(BUBBLES_IN_TUBE)
								PumpStepperRunStepSpeed_AbortReady(FW, Steps_tube_prime, Speed_ISE);
							PumpStepperRunStepSpeed_AbortReady(FW, runSteps_Clean, Speed_ISE);
							userDelay(valve_delay, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
							PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble, Speed_ISE);
							userDelay(valve_delay_after_air, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);
							PumpStepperRunStepSpeed_AbortReady(FW, runSteps_Clean_center, Speed_ISE);

							SleepValve();

							update_Status(STATUS_TEST, OPERATION_CL_ACTIVATION);
							if(REF_DRIFT != 0 && SATURATED_KCL_REF != 0)
								Ref_drift = Calculate_Ref_Drift(SATURATED_KCL_REF, MeasureTemperature(1));
							CleanAmperometrics_CurrentLimited(Ref_drift, 0, Test_Number, 0);

							// After running push solutions back slightly into pouch
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
							PumpStepperRunStepSpeed_AbortReady(FW, Steps_tube_bubble, Speed_ISE);
							userDelay(valve_delay_after_air, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);		// Move valve to air
							PumpStepperRunStepSpeed_AbortReady(BW, Steps_tube_bubble, Speed_ISE);
							userDelay(valve_delay, 1);
						}
					}

				//
				// Nitrite, B1 + C2
				//
				PrintTime();
				if((gui32Error & ABORT_ERRORS) == 0)
					if(MEASURE_NITRITE && ISEs.Config == PH_CL_CART)
					{
						DEBUG_PRINT(UARTprintf("Measuring Nitrite!\n");)
						update_Status(STATUS_TEST, OPERATION_SAMPLE_B2);

						// Prime B2 at beginning only once, don't push back until after done mixing
						if((gui32Error & ABORT_ERRORS) == 0)
						{
							// Prime a little C2 before test to clear out any contamination
							DEBUG_PRINT(UARTprintf("Priming C2... \n");)
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
							PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble, Speed_priming);
							userDelay(valve_delay_after_air, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_C2, VALVE_STEPS_PER_POSITION);
							PumpStepperRunStepSpeed_AbortReady(FW, Steps_tube_prime_buffers, Speed_priming);
							userDelay(valve_delay, 1);

							// Prime a little B1 before test to clear out any contamination
							DEBUG_PRINT(UARTprintf("Priming B1... \n");)
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
							PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble, Speed_priming);
							userDelay(valve_delay_after_air, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);
							PumpStepperRunStepSpeed_AbortReady(FW, Steps_tube_prime_buffers, Speed_priming);
							userDelay(valve_delay, 1);
						}

						// Fill channel with solution to be measured before metering measurment plug
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
						PumpStepperRunStepSpeed_AbortReady(FW, Steps_sample_rinse, Speed_priming);
						FindPossitionZeroPump();
						userDelay(valve_delay, 1);

						//
						// FCl, B1 Mixing
						//
						DEBUG_PRINT(UARTprintf("Mixing %d steps of B1 and %d steps of C2 into sample at the same time... \n", Steps_B1, Steps_C2);)
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
						PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble, Speed_Metering);
						if(CL_MIX_IN_AIR)
						{
							DEBUG_PRINT(UARTprintf("Pumping large air plug so arrays and reference are uncovered during mixing\n");)
							PumpStepperRunStepSpeed_AbortReady(FW, runSteps_Clean + 1000, Speed_placing);
						}
						userDelay(valve_delay_after_air, 1);

						// Pump buffer and solution
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
						PumpStepperRunStepSpeed_AbortReady(FW, Steps_PreB1, Speed_Metering);
						userDelay(valve_delay_metering, 1);

						RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
						PumpStepperRunStepSpeed_AbortReady(FW, Steps_B1, Speed_Metering);
						userDelay(valve_delay_metering, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
						PumpStepperRunStepSpeed_AbortReady(FW, (1000 - Steps_B1), Speed_Metering);
						userDelay(valve_delay_metering, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_C2, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
						PumpStepperRunStepSpeed_AbortReady(FW, Steps_C2, Speed_Metering);
						userDelay(valve_delay_metering, 1);

						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
						PumpStepperRunStepSpeed_AbortReady(FW, Steps_PostB1, Speed_placing);
						userDelay(valve_delay, 1);

						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
						PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble, Speed_placing);
						//						userDelay(valve_delay_after_air, 1);
						//						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to air
						//						PumpStepperRunStepSpeed_AbortReady(FW, Steps_follow_B1, Speed_placing);
						//						userDelay(valve_delay_metering, 1);

						PumpStepperMix(BW, Steps_cycles, Speed_mixing, mix_cycles);
						userDelay(diffusion_time, 1);

						PumpStepperRunStepSpeed_AbortReady(FW, Steps_center, Speed_placing);

						SleepValve();

						if((gui32Error & ABORT_ERRORS) == 0)
						{
							ConnectMemory(0);

							// Read ADC here to make sure it is working, I've seen ADC have problem during Cl read causing analog board to reset which threw off
							// Cl reading, by reading here hopefully we catch the problem and fix it before doing anything to the amperometric arrays
							ADCReadAvg(0, ADC4_CS_B, 5);

							// Turn on short and off parallel resistor to allow large current flows
							IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 1);	// Parallel switch must be on with short switch to work
							IO_Ext_Set(IO_EXT2_ADDR, 2, WORK_EL_SHORT, 1);
							if(HIGH_RANGE)
								IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_MID_CURRENT, 1);

							// Set reference for amperometric mode
							IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
							IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);

							// Connect all electrodes together for measuring
							IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 1);
							IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);

							DACVoltageSet(0, Nitrite_Amp_Voltage_Set, true);

							userDelay(3000, 1);			// Let run 3 seconds with short to allow large current at beginning

							IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 0);
							IO_Ext_Set(IO_EXT2_ADDR, 2, WORK_EL_SHORT, 0);	// Turn off short switch
							if(HIGH_RANGE)
								Nitrite_Samp_nA = *(CurrentTimeRead(0, ADC4_CS_B, 60, (int) Nitrite_Amp_Voltage_Set, 2, .02) + 1);	// nA
							else
								Nitrite_Samp_nA = *(CurrentTimeRead(0, ADC4_CS_B, 60, (int) Nitrite_Amp_Voltage_Set, 0, .02) + 1);	// nA

							// Let the working electrodes float when not measuring
							IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
							IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 0);
							if(HIGH_RANGE)
								IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_MID_CURRENT, 0);
							DACVoltageSet(0, 0, true);

							ConnectMemory(1);

							DEBUG_PRINT(UARTprintf("Nitrite Samp Only: %d nA * 1000\n", (int) (Nitrite_Samp_nA * 1000));)

							T_Nitrite_Samp = MeasureTemperature(1);
							DEBUG_PRINT(UARTprintf("Temp: %d C * 1000\n", (int) (T_Nitrite_Samp * 1000));)
						}

						// RE and CE floating
						IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);		// Leave RE floating
						IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);		// Leave CE floating

						// After running TCl push B2 buffer back slightly into pouch
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
						PumpStepperRunStepSpeed_AbortReady(FW, Steps_tube_bubble, Speed_Metering);
						userDelay(valve_delay_after_air, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to air
						PumpStepperRunStepSpeed_AbortReady(BW, Steps_tube_bubble, Speed_Metering);
						userDelay(valve_delay, 1);
						//					RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to air
						//					PumpStepperRunStepSlow_AbortReady(BW, Steps_tube_bubble);
						//					userDelay(valve_delay, 1);
						//					RunValveToPossition_Bidirectional_AbortReady(V_C2, VALVE_STEPS_PER_POSITION);		// Move valve to air
						//					PumpStepperRunStepSlow_AbortReady(BW, Steps_tube_bubble);
						//					userDelay(valve_delay, 1);
					}

				//
				// Clean a third time before measuring the standard addition
				//
				PrintTime();
				if((gui32Error & ABORT_ERRORS) == 0)
					if(CLEAN_AMPS_TEST && MEASURE_NITRITE && ISEs.Config == PH_CL_CART)
					{
						// Pump in rinse over amperometrics for cleaning
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
						FindPossitionZeroPump();
						userDelay(valve_delay_after_air, 1);
						DEBUG_PRINT(UARTprintf("Pumping cleaning solution to clean amperometrics!\n");)
						RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);

						if(BUBBLES_IN_TUBE)
							PumpStepperRunStepSpeed_AbortReady(FW, Steps_tube_prime, Speed_ISE);
						PumpStepperRunStepSpeed_AbortReady(FW, runSteps_Clean, Speed_ISE);
						userDelay(valve_delay, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble, Speed_ISE);
						userDelay(valve_delay_after_air, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);
						PumpStepperRunStepSpeed_AbortReady(FW, runSteps_Clean_center, Speed_ISE);

						SleepValve();

						if(REF_DRIFT != 0 && SATURATED_KCL_REF != 0)
							Ref_drift = Calculate_Ref_Drift(SATURATED_KCL_REF, MeasureTemperature(1));
						CleanAmperometrics_CurrentLimited(Ref_drift, 0, Test_Number, 0);

						// After running push solutions back slightly into pouch
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
						PumpStepperRunStepSpeed_AbortReady(FW, Steps_tube_bubble, Speed_Metering);
						userDelay(valve_delay_after_air, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);		// Move valve to air
						PumpStepperRunStepSpeed_AbortReady(BW, Steps_tube_bubble, Speed_Metering);
						userDelay(valve_delay, 1);
					}

				// Create variables for Cl mixing
				//
				// Nitrite, B2 + C2
				//
				PrintTime();
				if((gui32Error & ABORT_ERRORS) == 0)
					if(MEASURE_NITRITE && ISEs.Config == PH_CL_CART)
					{
						update_Status(STATUS_TEST, OPERATION_SAMPLE_B2);

						// Prime B2 at beginning only once, don't push back until after done mixing
						if((gui32Error & ABORT_ERRORS) == 0)
						{
							// Prime a little T1 before test to clear out any contamination
							DEBUG_PRINT(UARTprintf("Priming Nitrite from T1 port... \n");)
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
							PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble, Speed_Metering);
							userDelay(valve_delay_after_air, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);
							PumpStepperRunStepSpeed_AbortReady(FW, Steps_tube_prime_buffers, Speed_Metering);
							userDelay(valve_delay, 1);
						}

						// Fill channel with solution to be measured before metering out measurement plug
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
						PumpStepperRunStepSpeed_AbortReady(FW, Steps_sample_rinse, Speed_priming);
						FindPossitionZeroPump();
						userDelay(valve_delay, 1);

						//
						// FCl, B1 Mixing
						//
						DEBUG_PRINT(UARTprintf("Mixing %d steps of B1, %d steps of C2, and %d steps of NO2 all toghether...\n", Steps_B1, Steps_C2, Steps_B2);)
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
						PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble, Speed_Metering);
						if(CL_MIX_IN_AIR)
						{
							DEBUG_PRINT(UARTprintf("Pumping large air plug so arrays and reference are uncovered during mixing\n");)
							PumpStepperRunStepSpeed_AbortReady(FW, runSteps_Clean + 1000, Speed_placing);
						}
						userDelay(valve_delay_after_air, 1);

						// Pump buffer and solution
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
						PumpStepperRunStepSpeed_AbortReady(FW, Steps_PreB1, Speed_Metering);
						userDelay(valve_delay_metering, 1);

						//						RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
						//						PumpStepperRunStepSlow_AbortReady(FW, Steps_B1);
						//						userDelay(valve_delay_metering, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
						PumpStepperRunStepSpeed_AbortReady(FW, Steps_B1, Speed_Metering);
						userDelay(valve_delay_metering, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
						PumpStepperRunStepSpeed_AbortReady(FW, (1000 - Steps_B1), Speed_Metering);
						userDelay(valve_delay_metering, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_C2, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
						PumpStepperRunStepSpeed_AbortReady(FW, Steps_C2, Speed_Metering);
						userDelay(valve_delay_metering, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
						PumpStepperRunStepSpeed_AbortReady(FW, (1000 - Steps_C2), Speed_Metering);
						userDelay(valve_delay_metering, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
						PumpStepperRunStepSpeed_AbortReady(FW, Steps_B2, Speed_Metering);
						userDelay(valve_delay_metering, 1);

						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
						PumpStepperRunStepSpeed_AbortReady(FW, Steps_PostB1, Speed_placing);
						userDelay(valve_delay, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
						PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble, Speed_placing);
						//						userDelay(valve_delay_after_air, 1);
						//						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to air
						//						PumpStepperRunStepSpeed_AbortReady(FW, Steps_follow_B1, Speed_placing);
						//						userDelay(valve_delay_metering, 1);

						PumpStepperMix(BW, Steps_cycles, Speed_mixing, mix_cycles);
						userDelay(diffusion_time, 1);

						PumpStepperRunStepSpeed_AbortReady(FW, Steps_center, Speed_placing);

						SleepValve();

						if((gui32Error & ABORT_ERRORS) == 0)
						{
							ConnectMemory(0);

							// Read ADC here to make sure it is working, I've seen ADC have problem during Cl read causing analog board to reset which threw off
							// Cl reading, by reading here hopefully we catch the problem and fix it before doing anything to the amperometric arrays
							ADCReadAvg(0, ADC4_CS_B, 5);

							// Turn on short and off parallel resistor to allow large current flows
							IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 1);	// Parallel switch must be on with short switch to work
							IO_Ext_Set(IO_EXT2_ADDR, 2, WORK_EL_SHORT, 1);
							if(HIGH_RANGE)
								IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_MID_CURRENT, 1);

							// Set reference for amperometric mode
							IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
							IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 1);

							// Connect all electrodes together for measuring
							IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 1);
							IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 1);

							DACVoltageSet(0, Nitrite_Amp_Voltage_Set, true);

							userDelay(3000, 1);			// Let run 3 seconds with short to allow large current at beginning

							IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_HIGH_CURRENT, 0);
							IO_Ext_Set(IO_EXT2_ADDR, 2, WORK_EL_SHORT, 0);	// Turn off short switch
							if(HIGH_RANGE)
								Nitrite_SampNitrite_nA = *(CurrentTimeRead(0, ADC4_CS_B, 60, (int) Nitrite_Amp_Voltage_Set, 2, .02) + 1);	// nA
							else
								Nitrite_SampNitrite_nA = *(CurrentTimeRead(0, ADC4_CS_B, 60, (int) Nitrite_Amp_Voltage_Set, 0, .02) + 1);	// nA

							// Let the working electrodes float when not measuring
							IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWA, 0);
							IO_Ext_Set(IO_EXT1_ADDR, 3, WORK_EL_SWB, 0);
							if(HIGH_RANGE)
								IO_Ext_Set(IO_EXT2_ADDR, 3, WORK_EL_MID_CURRENT, 0);
							DACVoltageSet(0, 0, true);

							ConnectMemory(1);

							DEBUG_PRINT(UARTprintf("Nitrite Samp+Nitrite: %d nA * 1000\n", (int) (Nitrite_SampNitrite_nA * 1000));)

							T_Nitrite_SampNitrite = MeasureTemperature(1);
							DEBUG_PRINT(UARTprintf("Temp: %d C * 1000", (int) (T_Nitrite_SampNitrite * 1000));)
						}

						// RE and CE floating
						IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);		// Leave RE floating
						IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);		// Leave CE floating

						// After running TCl push B2 buffer back slightly into pouch
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
						PumpStepperRunStepSpeed_AbortReady(FW, runSteps_air_bubble + (3 * Steps_tube_bubble), Speed_Metering);
						userDelay(valve_delay_after_air, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to air
						PumpStepperRunStepSpeed_AbortReady(BW, Steps_tube_bubble, Speed_Metering);
						userDelay(valve_delay, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_C2, VALVE_STEPS_PER_POSITION);		// Move valve to air
						PumpStepperRunStepSpeed_AbortReady(BW, Steps_tube_bubble, Speed_Metering);
						userDelay(valve_delay, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_T1, VALVE_STEPS_PER_POSITION);		// Move valve to air
						PumpStepperRunStepSpeed_AbortReady(BW, Steps_tube_bubble, Speed_Metering);
						userDelay(valve_delay, 1);
					}
			}	// if MEASURE_NITRITE


			if((MEASURE_TCL || MEASURE_FCL || MEASURE_ALKALINITY || MEASURE_NITRITE) && (gui32Error & ABORT_ERRORS) == 0)	// Rinse with sample if chlorine was ran, skip if running alkalinity as it will be done there
#else
				if((MEASURE_TCL || MEASURE_FCL || MEASURE_ALKALINITY) && (gui32Error & ABORT_ERRORS) == 0)	// Rinse with sample if chlorine was ran, skip if running alkalinity as it will be done there

#endif	// MEASURE_NITRITE
			{
				//				for(i = 0; i < 5; i++)
				//				{
				//					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
				//					PumpStepperRunStepSpeed(FW, runSteps_air_bubble, Speed_ISE);
				//					userDelay(valve_delay,0);
				//					RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
				//					PumpStepperRunStepSpeed(FW, runSteps_air_bubble, Speed_ISE);
				//					userDelay(valve_delay_after_air,0);
				//				}
				//
				//				PumpStepperMix(FW, 2000, Speed_mixing, 20);

				//				RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
				//				PumpStepperRunStepSpeed_AbortReady(FW, 20000, Speed_priming);

				DEBUG_PRINT(UARTprintf("Rinsing with sample!\n");)
				PrintTime();

#if defined POST_BACK_AND_FORTH || defined POST_DIFFUSION_RINSE
				if(POST_BACK_AND_FORTH)
				{
					DEBUG_PRINT(UARTprintf("Rinsing by pumping back and forth!\n");)
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
					FindPossitionZeroPump();
					for (i = 0; i < Number_of_bubbles_samp; i++) // Loop over air/solution cycle 3 times for single solution
					{
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 1);
						userDelay(valve_delay_after_air, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
						PumpVolume(FW, PumpVol_Solution, Speed_Fast, 1);
						userDelay(valve_delay, 1);
					}

					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					PumpVolume(FW, PumpVol_Large_air_bubble + PumpVol_air_bubble, Speed_Fast, 1);
					userDelay(valve_delay_after_air, 1);

					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);

					PumpStepperMix(BW, 8000, Speed_Fast, 10);

					SleepValve();
				}
				else if(POST_DIFFUSION_RINSE)
				{
					DEBUG_PRINT(UARTprintf("Rinsing by letting several plugs sit over sensors!\n");)
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
					FindPossitionZeroPump();
					for (i = 0; i < Number_of_bubbles_samp; i++) // Loop over air/solution cycle 3 times for single solution
					{
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
						PumpVolume(FW, PumpVol_Sample + 134.4, Speed_Fast, 1);
						userDelay(valve_delay, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						PumpVolume(FW, PumpVol_plug_samp, Speed_Fast, 1);
						userDelay(10000, 1);	// Let solution sit over sensors for 5 seconds
					}

					SleepValve();
				}
				else
#endif
				{
					DEBUG_PRINT(UARTprintf("Rinsing with one long sample plug!\n");)
					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
					PumpVolume(FW, PumpVol_Sample_Prime, Speed_Fast, 1);
				}
			}

			uint8_t Storage_Port;
#ifdef STORE_HIGH_CONC_CAL
			DEBUG_PRINT(UARTprintf("Pumping Cal 5 as Postrinse\n");)
			Storage_Port = V_CAL_2;
#else
			/*if(ISEs.Config == PH_CL_CART && Sols->Cond_EEP_Cal_2 > 900)	// pH only cartridge with pH 9 clean in place of Cal 2
				Storage_Port = V_CAL_2;
			else */if((STORE_IN_CLEAN == 0 || (ISEs.Config == PH_CL_CART && STORE_PH6_CLEAN == 0)) && Sols->pH_EEP_Clean < 8.5)
				Storage_Port = V_RINSE;
			else
				Storage_Port = V_CLEAN;
#endif

			if(PURGE_SAMPLE)
			{
				if(CLEAR_SAMPLE_WITH_RINSE)
				{
					DEBUG_PRINT(UARTprintf("Purging sample tube by pushing rinse and air backwards into it\n");)
					RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
					PumpVolume(FW, PumpVol_sample_rinse, Speed_Fast, 0);
					userDelay(valve_delay_after_air, 0);
#ifdef CLEAR_SAMP_WITH_STORAGE_SOLTUION
					RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
#else
					RunValveToPossition_Bidirectional(V_RINSE, VALVE_STEPS_PER_POSITION);
#endif	// CLEAR_SAMP_WITH_STORAGE_SOLTUION
					PumpVolume(FW, 16.8 + PumpVol_tube_bubble, Speed_Fast, 0);
					userDelay(valve_delay, 0);
					RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
					PumpVolume(FW, 84, Speed_Fast, 0);
					userDelay(valve_delay_after_air, 0);
					RunValveToPossition_Bidirectional(V_SAMP, VALVE_STEPS_PER_POSITION);
					PumpVolume(BW, 302.4, Speed_Fast, 0);
					userDelay(valve_delay, 0);

					RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Push air so after a test the waste tube is empty
					PumpVolume(FW, PumpVol_Sample_Prime, Speed_Fast, 0);

#ifndef CLEAR_SAMP_WITH_STORAGE_SOLTUION
					if(Storage_Port != V_RINSE)	// Reset bubble back in Rinse port
					{
						RunValveToPossition_Bidirectional(V_RINSE, VALVE_STEPS_PER_POSITION);
						PumpVolume(BW, PumpVol_tube_bubble, Speed_Fast, 0);
						userDelay(valve_delay, 0);
					}
#endif	// CLEAR_SAMP_WITH_STORAGE_SOLTUION
				}
				else
				{
					DEBUG_PRINT(UARTprintf("Purging sample tube by pushing air backwards into it\n");)
					RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
					PumpVolume(FW, PumpVol_sample_rinse + PumpVol_Sample_Prime + 33.6, Speed_Fast, 0);
					userDelay(valve_delay_after_air, 0);
					RunValveToPossition_Bidirectional(V_SAMP, VALVE_STEPS_PER_POSITION);
					PumpVolume(BW, PumpVol_Sample_Prime, Speed_Fast, 0);
					userDelay(valve_delay, 0);
				}
			}
			else
			{
				DEBUG_PRINT(UARTprintf("Not purging sample tube\n");)
				RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
				PumpVolume(FW, PumpVol_sample_rinse, Speed_Fast, 0);
				userDelay(valve_delay_after_air, 0);
				if(BUBBLES_IN_TUBE)
				{
					RunValveToPossition_Bidirectional(V_SAMP, VALVE_STEPS_PER_POSITION);
					PumpVolume(BW, PumpVol_tube_bubble, Speed_Fast, 0);
					userDelay(valve_delay, 0);
				}
			}

#ifdef TESTING_MODE
			uint64_t flush_clock = TimerValueGet64(WTIMER0_BASE);
			DEBUG_PRINT(UARTprintf("Time to flush: %d\n", (uint32_t) ((flush_clock - total_clock) / SysCtlClockGet()));)
			DEBUG_PRINT(UARTprintf("Total time elapsed: %d s, or %d m %d s\n", (uint32_t) ((flush_clock - start_clock)/SysCtlClockGet()), (uint32_t) ((flush_clock - start_clock)/SysCtlClockGet())/60,(uint32_t) ((flush_clock - start_clock)/SysCtlClockGet())%60);)
#endif

			//
			// Flow Chart teal section, post-rinse
			//
			PrintTime();
//			if((gui32Error & ABORT_ERRORS) == 0)
			{
				update_Status(STATUS_TEST, OPERATION_TEST_POSTCHECK);
#ifdef CAL_2_RINSE
				DEBUG_PRINT(UARTprintf("Pumping Cal 2 as Postrinse... \n");)
#else
#ifdef PRINT_UART
				if(Storage_Port == V_CAL_2)	// pH only cartridge with pH 9 clean in place of Cal 2
					{DEBUG_PRINT(UARTprintf("Storing from Cal 2 Port\n");)}
				else if(Storage_Port == V_RINSE)
					{DEBUG_PRINT(UARTprintf("Pumping Postrinse\n");)}
				else
					{DEBUG_PRINT(UARTprintf("Pumping Clean as Postrinse\n");)}
#endif
#endif
				RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
				FindPossitionZeroPump();
				for (i = 0; i < Number_of_bubbles_Postrinse; i++) // Loop over air/solution cycle 3 times for single solution
				{
					RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 0);
					if(i == (Number_of_bubbles_Postrinse - 1))
						PumpVolume(FW, PumpVol_Large_air_bubble, Speed_Fast, 0);
					userDelay(valve_delay_after_air,0);
#ifdef CAL_2_RINSE
					RunValveToPossition_Bidirectional(V_CAL_2, VALVE_STEPS_PER_POSITION);
#else
					RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
#endif
					if(i == 0 && BUBBLES_IN_TUBE && CLEAR_SAMPLE_WITH_RINSE == 0)
						PumpVolume(FW, PumpVol_tube_bubble, Speed_Fast, 0);
					PumpVolume(FW, PumpVol_Solution, Speed_Fast, 0);
					if(i != (Number_of_bubbles_Postrinse - 1))
						userDelay(valve_delay, 0);

					if((gui32Error & ABORT_ERRORS) != 0)	// If abort command is received stop bubble cycle and just fill with rinse
						break;
				}
				if(STORE_HUMID == 1)
				{
					DEBUG_PRINT(UARTprintf("Storing with air over all sensors!\n");)
					RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					PumpVolume(FW, PumpVol_Solution + PumpVol_Rinse, Speed_Fast, 0);
					userDelay(valve_delay, 0);
#ifdef CAL_2_RINSE
					RunValveToPossition_Bidirectional(V_CAL_2, VALVE_STEPS_PER_POSITION);
#else
					RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
#endif
					PumpVolume(FW, PumpVol_Solution, Speed_Fast, 0);
				}
				else if(STORE_FRIT_DRY == 1)
				{
					DEBUG_PRINT(UARTprintf("Storing with air over reference!\n");)
//					userDelay(valve_delay, 1);
//					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
//					PumpStepperRunStepSpeed_AbortReady(FW, 3000, Speed_ISE);
//					userDelay(valve_delay, 1);
#ifdef CAL_2_RINSE
					RunValveToPossition_Bidirectional(V_CAL_2, VALVE_STEPS_PER_POSITION);
#else
					RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
#endif
					PumpVolume(FW, 134.4, Speed_Fast, 0);
					userDelay(valve_delay, 0);
				}
//				else if(STORE_AMPS_DRY == 1)
//				{
////					DEBUG_PRINT(UARTprintf("Storing air bubble over amperometrics only!\n");)
////					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
////					PumpStepperRunStepSpeed_AbortReady(FW, 2000, Speed_ISE);
////					userDelay(valve_delay, 1);
////					RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);
////					PumpStepperRunStepSpeed_AbortReady(FW, 11360, Speed_ISE);
////					userDelay(valve_delay, 1);
//
//					DEBUG_PRINT(UARTprintf("Storing air bubble over amperometrics only!\n");)
//					RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
//					PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 0);
//					userDelay(valve_delay, 1);
//#ifdef CAL_2_RINSE
//					RunValveToPossition_Bidirectional(V_CAL_2, VALVE_STEPS_PER_POSITION);
//#else
//					RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
//#endif
//					PumpVolume(FW, 117.6, Speed_Fast, 0);
//					userDelay(valve_delay, 0);
//
//					// Set RE and CE floating and close RE/CE loop
//					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
//					IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
//
//					// 10.7 uApp R = 309k + 499k = 808k
//					IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
//					IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);
//
//					WaveGenSet(1);
//					userDelay(100, 1);
//					uint8_t signal_doubled = 0;
//					uint8_t checkstep = 0;
//
//					float CondReading = ConductivityMovingAvg(COND_FREQ);
//					float OldReading = CondReading;
//
//					DEBUG_PRINT(UARTprintf("Reading\t%d\n", (int) CondReading);)
//					while(signal_doubled == 0 && checkstep <= 60)
//					{
////						if((checkstep + 5) % 10 == 0)
////						{
////							PumpStepperRunStepSpeed(FW, 500, Speed_ISE);
////							checkstep += 5;
////						}
////						else
////						{
////							PumpStepperRunStepSpeed(FW, 100, Speed_ISE);
////							checkstep++;
////						}
//
//						PumpVolume(FW, 2.75, Speed_Fast, 0);
//						checkstep++;
//
//						CondReading = ConductivityMovingAvg(COND_FREQ);
//						DEBUG_PRINT(UARTprintf("Reading\t%d\n", (int) CondReading);)
//
//						if(CondReading/OldReading > 2)
//						{
//							DEBUG_PRINT(UARTprintf("Conductivity signal doubled! Found storage location!\n");)
//							signal_doubled = 1;
//						}
//						OldReading = CondReading;
//
//
//					}
//					if(checkstep > 37)
//					{
//						DEBUG_PRINT(UARTprintf("Didn't find storage location!\n");)
//						PumpVolume(FW, 33.6, Speed_Fast, 0);
//					}
//
//					WaveGenSet(0);
//				}
				else
				{
					DEBUG_PRINT(UARTprintf("Storing covering everyting!\n");)
					PumpVolume(FW, PumpVol_Rinse, Speed_Fast, 0);
					if(FLOOD_TO_STORE == 0)
					{
						userDelay(valve_delay, 0);
						RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					}
					PumpVolume(FW, PumpVol_plug, Speed_Fast, 0);
				}
				//
				//				if(STORE_HUMID == 0)
				//				{
				//					PumpStepperRunStepSpeed_AbortReady(FW, runSteps_PostRinse, Speed_ISE);
				//					if(FLOOD_TO_STORE == 0)
				//					{
				//						userDelay(valve_delay, 1);
				//						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
				//					}
				//					PumpStepperRunStepSpeed_AbortReady(FW, runSteps_plug, Speed_ISE);
				//				}
				//				else
				//				{
				//						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
				//						PumpStepperRunStepSpeed_AbortReady(FW, runSteps_Solution_Postrinse + runSteps_PostRinse, Speed_ISE);
				//						userDelay(valve_delay, 1);
				//						RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);
				//						PumpStepperRunStepSpeed_AbortReady(FW, runSteps_plug, Speed_ISE);
				//				}


				//				PumpStepperRunStepSpeed(FW, runSteps_PostRinse, Speed_ISE);
				//				userDelay(valve_delay,0);
				//				if(FLOOD_TO_STORE == 0)
				//					RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
				//				PumpStepperRunStepSpeed(FW, runSteps_plug, Speed_ISE);

				SleepValve();

				Sensor_in_rinse = 1;
			}

			if(MEASURE_POSTRINSE)
			{
				float ISE_mV_Postrinse[10];
				CollectISEmV(ISE_mV_Postrinse, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);
#ifdef RESET_ELECTRONICS_READ
				if(RESET_ELECTRONICS_READ)
				{
					AnalogOff();
					userDelay(20000, 1);
					InitAnalog();

					CollectISEmV(ISE_mV_Postrinse, 0xFFFF, ISE_WAIT, PRINT_ISE_TIME_DATA, &ISEs);
				}
#endif

				float T_PostRinse = MeasureTemperature(1);

				// Set RE and CE floating and close RE/CE loop
				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);

#ifdef PRINT_UART
				float Conductivity_PostRinse;// = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);
#endif
#ifndef COND_SOLUTION_STRUCT
				if(Sols->pH_EEP_Cal_2 < 9) // This is Cal 3, not Cal 2
				{
					if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
						Conductivity_PostRinse = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_1, 0);
					else
						Conductivity_PostRinse = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_1, 0);
				}
				else
				{
					if(Sols->Cond_EEP_Clean == Sols->Cond_EEP_Clean)
						Conductivity_PostRinse = MeasureConductivity(Sols->Cond_EEP_Clean, Sols->Cond_EEP_Cal_2, 0);
					else
						Conductivity_PostRinse = MeasureConductivity(Sols->Cond_EEP_Rinse, Sols->Cond_EEP_Cal_2, 0);
				}
#else
#ifdef PRINT_UART
				Conductivity_PostRinse = MeasureConductivity(Sols, 0);
#endif
#endif

#ifdef PRINT_UART
				DEBUG_PRINT(UARTprintf("Rinse Conductivity Measured at: %d\n", (int) (Conductivity_PostRinse * 1000));)
				if(ISEs.Config == PH_CL_CART)
				{
					DEBUG_PRINT(UARTprintf("Postrinse temp and corrected conductivity: =%d/1000\t=%d/1000\n", (int) (T_PostRinse * 1000), (int) ((Conductivity_PostRinse / (1 + Sols->Clean_Cond_TComp*(T_PostRinse - 25))) * 1000));)
				}
				else
				{
					DEBUG_PRINT(UARTprintf("Postrinse temp and corrected conductivity: =%d/1000\t=%d/1000\n", (int) (T_PostRinse * 1000), (int) ((Conductivity_PostRinse / (1 + Sols->Rinse_Cond_TComp*(T_PostRinse - 25))) * 1000));)
				}
				DEBUG_PRINT(UARTprintf("Rinse Conductivity saved on memory: %d\n", (int) (Sols->Cond_EEP_Rinse * 1000));)

				DEBUG_PRINT(UARTprintf("Rinse - Postrinse mV:\n");)
				for(i = 0; i < 10; i++)
					{DEBUG_PRINT(UARTprintf("=%d/1000\t", (int) ((ISE_E_Rinse[i] - ISE_mV_Postrinse[i]) * 1000));)}
				DEBUG_PRINT(UARTprintf("=%d/1000\n", (int) ((Conductivity_Rinse - Conductivity_PostRinse) * 1000));)

				DEBUG_PRINT(UARTprintf("\n");)
#endif
			}

			// RE and CE floating
			IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);		// Leave RE floating
			IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);		// Leave CE floating

			// Push air back into rinse port before moving to next solution
			if(BUBBLES_IN_TUBE)
			{
				RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
//				if((STORE_IN_CLEAN == 0 || (ISEs.Config == PH_CL_CART && STORE_PH6_CLEAN == 0)) && Sols->pH_EEP_Clean < 8.5)
					PumpVolume(FW, PumpVol_tube_bubble * 4, Speed_Fast, 0);
//				else
//					PumpVolume(FW, PumpVol_tube_bubble * 6, Speed_Fast, 0);
				userDelay(valve_delay_after_air, 0);
#ifdef CAL_2_RINSE
				RunValveToPossition_Bidirectional(V_CAL_2, VALVE_STEPS_PER_POSITION);
#else
				RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
#endif
				PumpVolume(BW, PumpVol_tube_bubble, Speed_Fast, 0);
				userDelay(valve_delay, 0);
			}

			if(Sensor_in_rinse == 0)	// This happens when abort command is received while running test
			{
				DEBUG_PRINT(UARTprintf("Abort received storing sensor!\n");)
				//
				// Flow Chart teal section, post-rinse
				//
				PrintTime();
					update_Status(STATUS_TEST, OPERATION_TEST_POSTCHECK);
	#ifdef CAL_2_RINSE
					DEBUG_PRINT(UARTprintf("Pumping Cal 2 as Postrinse... \n");)
	#else
					DEBUG_PRINT(UARTprintf("Pumping Postrinse... \n");)
	#endif
					RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
					FindPossitionZeroPump();
					for (i = 0; i < 1; i++) // Loop over air/solution cycle 3 times for single solution
					{
						RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 0);
						if(i == (Number_of_bubbles_Postrinse - 1))
							PumpVolume(FW, PumpVol_Large_air_bubble, Speed_Fast, 0);
						userDelay(valve_delay_after_air,0);
	#ifdef CAL_2_RINSE
						RunValveToPossition_Bidirectional(V_CAL_2, VALVE_STEPS_PER_POSITION);
	#else
						RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
	#endif
						if(i == 0 && BUBBLES_IN_TUBE && CLEAR_SAMPLE_WITH_RINSE == 0)
							PumpVolume(FW, PumpVol_Solution + PumpVol_tube_bubble, Speed_Fast, 0);
						else
							PumpVolume(FW, PumpVol_Solution, Speed_Fast, 0);
						if(i != (Number_of_bubbles_Postrinse - 1))
							userDelay(valve_delay, 0);

						if((gui32Error & ABORT_ERRORS) != 0)	// If abort command is received stop bubble cycle and just fill with rinse
							break;
					}
					if(STORE_HUMID == 1)
					{
						DEBUG_PRINT(UARTprintf("Storing with air over all sensors!\n");)
						RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						PumpVolume(FW, PumpVol_Solution + PumpVol_Rinse, Speed_Fast, 0);
						userDelay(valve_delay, 0);
	#ifdef CAL_2_RINSE
						RunValveToPossition_Bidirectional(V_CAL_2, VALVE_STEPS_PER_POSITION);
	#else
						RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
	#endif
						PumpVolume(FW, PumpVol_Solution, Speed_Fast, 0);
					}
					else if(STORE_FRIT_DRY == 1)
					{
						DEBUG_PRINT(UARTprintf("Storing with air over amps and reference!\n");)
	//					userDelay(valve_delay, 1);
	//					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
	//					PumpStepperRunStepSpeed_AbortReady(FW, 3000, Speed_ISE);
	//					userDelay(valve_delay, 1);
	#ifdef CAL_2_RINSE
						RunValveToPossition_Bidirectional(V_CAL_2, VALVE_STEPS_PER_POSITION);
	#else
						RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
	#endif
						PumpVolume(FW, 134.4, Speed_Fast, 0);
					}
//					else if(STORE_AMPS_DRY == 1)
//					{
//						DEBUG_PRINT(UARTprintf("Storing air bubble over amperometrics only!\n");)
//						RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
//						PumpVolume(FW, 33.6, Speed_Fast, 0);
//						userDelay(valve_delay, 0);
//	#ifdef CAL_2_RINSE
//						RunValveToPossition_Bidirectional(V_CAL_2, VALVE_STEPS_PER_POSITION);
//	#else
//						RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
//	#endif
//						PumpVolume(FW, 117.6, Speed_Fast, 0);
//						userDelay(valve_delay, 0);
//
//						// Set RE and CE floating and close RE/CE loop
//						IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
//						IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
//
//						// 10.7 uApp R = 309k + 499k = 808k
//						IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
//						IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);
//
//						WaveGenSet(1);
//						userDelay(100, 0);
//						uint8_t signal_doubled = 0;
//						uint8_t checkstep = 0;
//
//						float CondReading = ConductivityMovingAvg(COND_FREQ);
//						float OldReading = CondReading;
//
//						DEBUG_PRINT(UARTprintf("Reading\t%d\n", (int) CondReading);)
//						while(signal_doubled == 0 && checkstep <= 60)
//						{
////							if((checkstep + 5) % 10 == 0)
////							{
////								PumpStepperRunStepSpeed(FW, 500, Speed_ISE);
////								checkstep += 5;
////							}
////							else
////							{
////								PumpStepperRunStepSpeed(FW, 100, Speed_ISE);
////								checkstep++;
////							}
//
//							PumpVolume(FW, 2.75, Speed_Fast, 0);
//							checkstep++;
//
//							CondReading = ConductivityMovingAvg(COND_FREQ);
//							DEBUG_PRINT(UARTprintf("Reading\t%d\n", (int) CondReading);)
//
//							if(CondReading/OldReading > 2)
//							{
//								DEBUG_PRINT(UARTprintf("Conductivity signal doubled! Found storage location!\n");)
//								signal_doubled = 1;
//							}
//							OldReading = CondReading;
//
//
//						}
//						if(checkstep > 37)
//						{
//							DEBUG_PRINT(UARTprintf("Didn't find storage location!\n");)
//							PumpVolume(FW, 33.6, Speed_Fast, 0);
//						}
//
//						WaveGenSet(0);
//					}
					else
					{
						DEBUG_PRINT(UARTprintf("Storing covering everything!\n");)
						PumpVolume(FW, PumpVol_Rinse, Speed_Fast, 0);
						if(FLOOD_TO_STORE == 0)
						{
							userDelay(valve_delay, 0);
							RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						}
						PumpVolume(FW, PumpVol_plug, Speed_Fast, 0);
					}

					SleepValve();

					Sensor_in_rinse = 1;

				// RE and CE floating
				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);		// Leave RE floating
				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);		// Leave CE floating

				// Push air back into rinse port before moving to next solution
				if(BUBBLES_IN_TUBE)
				{
					RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
//					if((STORE_IN_CLEAN == 0 || (ISEs.Config == PH_CL_CART && STORE_PH6_CLEAN == 0)) && Sols->pH_EEP_Clean < 8.5)
						PumpVolume(FW, PumpVol_tube_bubble * 4, Speed_Fast, 0);
//					else
//						PumpVolume(FW, PumpVol_tube_bubble * 6, Speed_Fast, 0);
					userDelay(valve_delay_after_air, 0);
	#ifdef CAL_2_RINSE
					RunValveToPossition_Bidirectional(V_CAL_2, VALVE_STEPS_PER_POSITION);
	#else
					RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
	#endif
					PumpVolume(BW, PumpVol_tube_bubble, Speed_Fast, 0);
					userDelay(valve_delay, 0);
				}
			}

			SleepValve();

#ifdef TESTING_MODE
			uint64_t store_clock = TimerValueGet64(WTIMER0_BASE);
			DEBUG_PRINT(UARTprintf("Time to flush: %d\n", (uint32_t) ((store_clock - flush_clock) / SysCtlClockGet()));)
			DEBUG_PRINT(UARTprintf("Total time elapsed: %d s, or %d m %d s\n", (uint32_t) ((store_clock - start_clock)/SysCtlClockGet()), (uint32_t) ((store_clock - start_clock)/SysCtlClockGet())/60,(uint32_t) ((store_clock - start_clock)/SysCtlClockGet())%60);)
#endif

			if((gui32Error & ABORT_ERRORS) == 0)
			{
				// Track number of tests each sensor has performed in Sensor Usage characteristic
				uint16_t No_of_tests = *((uint16_t *) MemoryRead(PAGE_CARTRIDGE_INFO, OFFSET_COMPLETED_TESTS, 2));
				if(No_of_tests == 0xFFFF)
					No_of_tests = 0;
				No_of_tests++;
				MemoryWrite(PAGE_CARTRIDGE_INFO, OFFSET_COMPLETED_TESTS, 2, (uint8_t *) &No_of_tests);

#ifdef PRINT_UART
				if(PRINT_RAW == 1)
				{
					// Print out raw data
					DEBUG_PRINT(UARTprintf("Raw Data: \n");)

					DEBUG_PRINT(UARTprintf("Sensor\tPrerinse\tSample\n");)
					for(i = 0; i < ISEs.pH_H2.size; i++)
						{DEBUG_PRINT(UARTprintf("pH H2 %d\t%d\t%d\n", i + 1, (int) (pH_H2_E_Rinse[i] * 1000), (int) (pH_H2_E_Samp[i] * 1000));)}
					for(i = 0; i < ISEs.pH_Cr.size; i++)
						{DEBUG_PRINT(UARTprintf("pH Cr %d\t%d\t%d\n", i + 1, (int) (pH_Cr_E_Rinse[i] * 1000), (int) (pH_Cr_E_Samp[i] * 1000));)}
					for(i = 0; i < ISEs.TH.size; i++)
						{DEBUG_PRINT(UARTprintf("TH %d\t%d\t%d\n", i + 1, (int) (TH_E_Rinse[i] * 1000), (int) (TH_E_Samp[i] * 1000));)}
					for(i = 0; i < ISEs.NH4.size; i++)
						{DEBUG_PRINT(UARTprintf("NH4 %d\t%d\t%d\n", i + 1, (int) (NH4_E_Rinse[i] * 1000), (int) (NH4_E_Samp[i] * 1000));)}
					for(i = 0; i < ISEs.Ca.size; i++)
						{DEBUG_PRINT(UARTprintf("Ca %d\t%d\t%d\n", i + 1, (int) (Ca_E_Rinse[i] * 1000), (int) (Ca_E_Samp[i] * 1000));)}

					DEBUG_PRINT(UARTprintf("Temperatures:\n");)
					DEBUG_PRINT(UARTprintf("Prerinse\t%d\tC * 1000\n", (int) (T_Rinse * 1000));)
					DEBUG_PRINT(UARTprintf("Sample\t%d\tC * 1000\n", (int) (T_Samp * 1000));)
					DEBUG_PRINT(UARTprintf("Samp + B1\t%d\tC * 1000\n", (int) (T_Samp_B1 * 1000));)
					DEBUG_PRINT(UARTprintf("Samp + B2\t%d\tC * 1000\n", (int) (T_Samp_B2 * 1000));)
					//					DEBUG_PRINT(UARTprintf("Postrinse\t%d\tC * 1000\n", (int) (T_PostRinse * 1000));)

					// Print out raw data
					float ConductivityReading = Build_float(MemoryRead(Test_page, OFFSET_RAW_COND, 4));
					DEBUG_PRINT(UARTprintf("Raw Conductivity Data: \n");)
					DEBUG_PRINT(UARTprintf("Cond \t%d\t uV * 1000 \n", (int) (ConductivityReading * 1000));)
					//					DEBUG_PRINT(UARTprintf("CondReg \t%d\n", ConductivityRegion);

					DEBUG_PRINT(UARTprintf("Raw Chlorine Data:\n");)
					if(MEASURE_FCL)
						{DEBUG_PRINT(UARTprintf("FCL \t %d \t nA * 1000\n", (int) (Cl_nA_FCl * 1000));)}
					if(MEASURE_TCL)
						{DEBUG_PRINT(UARTprintf("TCL \t %d \t nA * 1000\n", (int) (Cl_nA_TCl * 1000));)}
				}

				DEBUG_PRINT(UARTprintf("\n");)

				DEBUG_PRINT(UARTprintf("Calculated values:\n");)

				for(i = 0; i < ISEs.pH_H2.size; i++)
					DEBUG_PRINT(UARTprintf("pH (H2) %d\t", i + 1);)
				for(i = 0; i < ISEs.pH_Cr.size; i++)
					DEBUG_PRINT(UARTprintf("pH (Cr) %d\t", i + 1);)
				for(i = 0; i < ISEs.TH.size; i++)
					DEBUG_PRINT(UARTprintf("TH %d\t", i + 1);)
				for(i = 0; i < ISEs.NH4.size; i++)
					DEBUG_PRINT(UARTprintf("NH4 %d\t", i + 1);)
				for(i = 0; i < ISEs.Ca.size; i++)
					DEBUG_PRINT(UARTprintf("Ca %d\t", i + 1);)
				DEBUG_PRINT(UARTprintf("Conductivity\tTherm Temp\tRS Temp");)
				DEBUG_PRINT(UARTprintf("\n");)

				for(i = 0; i < ISEs.pH_H2.size; i++)
					DEBUG_PRINT(UARTprintf("=%d/1000\t", (int) (pH_H2_Samp[i] * 1000));)
				for(i = 0; i < ISEs.pH_Cr.size; i++)
					DEBUG_PRINT(UARTprintf("=%d/1000\t", (int) (pH_Cr_Samp[i] * 1000));)
				for(i = 0; i < ISEs.TH.size; i++)
					DEBUG_PRINT(UARTprintf("=%d/1000\t", (int) (TH_corr[i] * 1000));)
				for(i = 0; i < ISEs.NH4.size; i++)
					DEBUG_PRINT(UARTprintf("=%d/1000\t", (int) (NH4_NH3_N_Free[i] * 1000));)
				for(i = 0; i < ISEs.Ca.size; i++)
					DEBUG_PRINT(UARTprintf("=%d/1000\t", (int) (Ca_Hardness[i] * 1000));)
				DEBUG_PRINT(UARTprintf("=%d/1000\t", (int) (Conductivity * 1000));)
				DEBUG_PRINT(UARTprintf("=%d/1000\t", (int) (T_Therm * 1000));)
				DEBUG_PRINT(UARTprintf("=%d/1000", (int) (T_RS * 1000));)

				DEBUG_PRINT(UARTprintf("\n\n");)

				DEBUG_PRINT(UARTprintf("Calculated values:\n");)
				for(i = 0; i < ISEs.pH_H2.size; i++)
					DEBUG_PRINT(UARTprintf("pH H2 %d\t%d\n", i + 1, (int) (pH_H2_Samp[i] * 1000));)
				for(i = 0; i < ISEs.pH_Cr.size; i++)
					DEBUG_PRINT(UARTprintf("pH Cr %d\t%d\n", i + 1, (int) (pH_Cr_Samp[i] * 1000));)
				for(i = 0; i < ISEs.TH.size; i++)
					DEBUG_PRINT(UARTprintf("TH Fixed Log K %d\t%d\n", i + 1, (int) (TH_corr[i] * 1000));)
#ifdef TH_ITERATED_MATH
				if(ISEs.TH.size > 0)
				{
//					for(i = 0; i < 2; i++)
//						DEBUG_PRINT(UARTprintf("TH Nick's Math %d\t%d\tLog k %d/1000\n", i + 1, (int) (TH_iterated[i] * 1000), (int) (log_K_Ca_Mg_Nick[i] * 1000));)
					for(i = 0; i < 2; i++)
						DEBUG_PRINT(UARTprintf("TH Ratio Ramp Math %d\t%d\n", i + 1, (int) (TH_iterated[i] * 1000));)
				}
#endif
				for(i = 0; i < ISEs.NH4.size; i++)
					DEBUG_PRINT(UARTprintf("NH4 %d\t%d\n", i + 1, (int) (NH4_NH3_N_Free[i] * 1000));)
				//				for(i = 0; i < ui8Size_NH4; i++)
				//					DEBUG_PRINT(UARTprintf("NH4 + T1 %d\t%d\n", i + 1, (int) (NH4_NH3_N_Total_T1[i] * 1000));)
				for(i = 0; i < ISEs.Ca.size; i++)
					DEBUG_PRINT(UARTprintf("Ca %d\t%d\n", i + 1, (int) (Ca_Hardness[i] * 1000));)
				if(ISEs.RunAlk)
				{
					for(i = 0; i < ISEs.pH_H2.size; i++)
						DEBUG_PRINT(UARTprintf("Alkalinity H2 %d\t%d\n", i + 1, (int) (Alk_Samp[i + ISEs.pH_H2.index] * 1000));)

					for(i = 0; i < ISEs.pH_Cr.size; i++)
						DEBUG_PRINT(UARTprintf("Alkalinity Cr %d\t%d\n", i + 1, (int) (Alk_Samp[i + ISEs.pH_Cr.index] * 1000));)
				}

				if(MEASURE_FCL)
					DEBUG_PRINT(UARTprintf("FCl \t %d \t ppm Cl2 * 1000 \n", (int) (Cl_FCl_ppm * 1000));)
				if(MEASURE_TCL)
					DEBUG_PRINT(UARTprintf("TCl \t %d \t ppm Cl2 * 1000 \n", (int) (Cl_TCl_ppm * 1000));)
				if(MEASURE_FCL && MEASURE_TCL)
				{
					DEBUG_PRINT(UARTprintf("MCl \t %d \t ppm Cl2 * 1000 \n", (int) (Cl_MCl_ppm * 1000));)
					//					DEBUG_PRINT(UARTprintf("MCl \t %d \t ppm N-NH3 * 1000 \n", (int) (Cl_MCl_NH3 * 1000));)
				}
				DEBUG_PRINT(UARTprintf("Conductivity \t%d\tuS/cm * 1000\n", (int) (Conductivity * 1000));)
				DEBUG_PRINT(UARTprintf("ORP\t%d\n", (int) ORP);)
				DEBUG_PRINT(UARTprintf("\n");)

				DEBUG_PRINT(UARTprintf("Sensors picked: \n");)
				DEBUG_PRINT(UARTprintf("pH\t%d\n", T_Chosen_pH);)
				if(ISEs.TH.size > 0)
					DEBUG_PRINT(UARTprintf("TH Fixed Log K\t%d\n", T_Chosen_TH);)
				if(ISEs.TH.size > 0)
					DEBUG_PRINT(UARTprintf("TH Ratio Ramp\t%d\n", T_Chosen_TH_RR);)
				if(ISEs.NH4.size > 0)
					DEBUG_PRINT(UARTprintf("NH4\t%d\n", T_Chosen_NH4);)
				if(ISEs.Ca.size > 0)
					DEBUG_PRINT(UARTprintf("Ca\t%d\n", T_Chosen_Ca);)
				if(ISEs.RunAlk)
					DEBUG_PRINT(UARTprintf("Alk\t%d\n", T_Chosen_Alk);)
				DEBUG_PRINT(UARTprintf("\n");)

				// Print out calculations using chosen sensors to console
				DEBUG_PRINT(UARTprintf("Values calculated using chosen sensors: \n");)
				DEBUG_PRINT(UARTprintf("pH\t%d\tpH * 1000\n", (int) (pH_Cr_Samp[T_Chosen_pH] * 1000));)
#ifdef REPORT_TH_RATIO_RAMP
				if(ISEs.TH.size > 0)
					DEBUG_PRINT(UARTprintf("Total Hardness\t%d\tCaCO3 ppm * 1000 \n", (int) (TH_iterated[T_Chosen_TH_RR] * 1000));)
#else
				if(ISEs.TH.size > 0)
					DEBUG_PRINT(UARTprintf("Total Hardness\t%d\tCaCO3 ppm * 1000 \n", (int) (TH_corr[T_Chosen_TH] * 1000));)
#endif
				if(ISEs.NH4.size > 0)
				{
					DEBUG_PRINT(UARTprintf("Total NH4 + NH3\t%d\tN-NH3 ppm * 1000 \n", (int) (NH4_NH3_N_Free[T_Chosen_NH4] * 1000));)
					DEBUG_PRINT(UARTprintf("Total NH4 + NH3 + MCl\t%d\tN-NH3 ppm * 1000 \n", (int) (NH4_TNH3 * 1000));)
				}
				//				DEBUG_PRINT(UARTprintf("Total NH4 + NH3 + T1\t%d\tN-NH3 ppm * 1000 \n", (int) (NH4_NH3_N_Total_T1[T_Chosen_NH4] * 1000));)
				if(ISEs.Ca.size > 0)
					DEBUG_PRINT(UARTprintf("Ca Hardness\t%d\tCaCO3 ppm * 1000 \n", (int) (Ca_Hardness[T_Chosen_Ca] * 1000));)
				if(MEASURE_ALKALINITY == 1 && ISEs.RunAlk)
					DEBUG_PRINT(UARTprintf("Alkalinkity\t%d\n", (int) (Alk_Samp[T_Chosen_Alk] * 1000));)

				if(MEASURE_FCL)
					DEBUG_PRINT(UARTprintf("Free Cl2\t%d\tCl2 ppm * 1000\n", (int) (Cl_FCl_ppm * 1000));)
				if(MEASURE_TCL)
					DEBUG_PRINT(UARTprintf("Total Cl2\t%d\tCl2 ppm * 1000\n", (int) (Cl_TCl_ppm * 1000));)

				DEBUG_PRINT(UARTprintf("Conductivity\t%d\tuS/cm * 1000\n", (int) (Conductivity * 1000));)
				DEBUG_PRINT(UARTprintf("\n");)

				if(MEASURE_FCL || MEASURE_TCL)
					DEBUG_PRINT(UARTprintf("Raw Chlorine Data:\n");)
				if(MEASURE_FCL)
					DEBUG_PRINT(UARTprintf("FCL \t %d \t nA * 1000\n", (int) (Cl_nA_FCl * 1000));)
				if(MEASURE_TCL)
					DEBUG_PRINT(UARTprintf("TCL \t %d \t nA * 1000\n\n", (int) (Cl_nA_TCl * 1000));)

				if(MEASURE_ALKALINITY == 1 && ISEs.RunAlk)
					DEBUG_PRINT(UARTprintf("TH Ca Relationship Math\t%d\n", (int) ((1.4817 * (Ca_Hardness[T_Chosen_Ca])) * 1000));)

#ifdef MEASURE_NITRITE
				if(MEASURE_NITRITE)
				{
					DEBUG_PRINT(UARTprintf("Temperatures:\n");)
					//					DEBUG_PRINT(UARTprintf("Blank 1: %d\n", (int) (T_Blank_1 * 1000));)
					//					DEBUG_PRINT(UARTprintf("Blank 2: %d\n", (int) (T_Blank_2 * 1000));)
					DEBUG_PRINT(UARTprintf("Nitrite Sample Only: %d\n", (int) (T_Nitrite_Samp * 1000));)
					DEBUG_PRINT(UARTprintf("Nitrite Sample Addition: %d\n\n", (int) (T_Nitrite_SampNitrite * 1000));)

					//					DEBUG_PRINT(UARTprintf("Nitrite Blank 1 nA: %d nA * 1000\n", (int) (Nitrite_Blank_1 * 1000));)
					//					DEBUG_PRINT(UARTprintf("Nitrite Blank 2 nA: %d nA * 1000\n", (int) (Nitrite_Blank_2 * 1000));)
					DEBUG_PRINT(UARTprintf("Nitrite Sample Only nA: %d nA * 1000\n", (int) (Nitrite_Samp_nA * 1000));)
					DEBUG_PRINT(UARTprintf("Nitrite Sample Addition nA: %d nA * 1000\n", (int) (Nitrite_SampNitrite_nA * 1000));)

					float Nitrite_Blank = Build_float(MemoryRead(PAGE_FACTORY_CAL, OFFSET_NITRITE_BLANK, 4));
					float Nitrite_SA = Build_float(MemoryRead(PAGE_SOLUTIONS, OFFSET_NITRITE_SA, 4));
					if(Nitrite_Blank != Nitrite_Blank)
					{
						Nitrite_Blank = 11.5;
						DEBUG_PRINT(UARTprintf("No nitrite blank saved in memory setting to: %d\n", (int) Nitrite_Blank);)
					}
					if(Nitrite_SA != Nitrite_SA)
					{
						Nitrite_SA = 13.8;
						DEBUG_PRINT(UARTprintf("No nitrite standard addition saved in memory setting to: %d\n", (int) Nitrite_SA);)
					}

					//				float Nitrite_Blank = 11.5;
					//					float Nitrite_Blank_Avg = (Nitrite_Blank_1 + Nitrite_Blank_2) / 2;
					//					float Nitrite_SA = 13.8;
					float Nitrite_adj_1 = (Nitrite_Samp_nA - Nitrite_Blank) * 0.859;
					float Nitrite_adj_2 = (Nitrite_SampNitrite_nA - Nitrite_Blank);
					float Nitrite_Slope = (Nitrite_adj_1 - Nitrite_adj_2) / (-1 * Nitrite_SA * 0.112);
					float Nitrite_sample = Nitrite_adj_1 * 1.401 / Nitrite_Slope;

					//					MemoryWrite(Test_page, OFFSET_TEST_NITRITE_BLANK_1_NA, 4, (uint8_t *) &Nitrite_Blank_1);
					//					MemoryWrite(Test_page, OFFSET_TEST_NITRITE_BLANK_2_NA, 4, (uint8_t *) &Nitrite_Blank_2);

					MemoryWrite(Test_page, OFFSET_TEST_NITRITE_BACK_NA, 4, (uint8_t *) &Nitrite_Samp_nA);
					MemoryWrite(Test_page, OFFSET_TEST_NITRITE_NA, 4, (uint8_t *) &Nitrite_SampNitrite_nA);
					MemoryWrite(Test_page, OFFSET_TEST_NITRITE, 4, (uint8_t *) &Nitrite_sample);

					DEBUG_PRINT(UARTprintf("Calculated Nitrite: %d ppm * 1000\n", (int) (Nitrite_sample * 1000));)
				}
#endif	// Measure Nitrite

				DEBUG_PRINT(UARTprintf("Temp Thermistor: %d C * 1000\n", (int) (T_Therm * 1000));)
				DEBUG_PRINT(UARTprintf("Temp Amps: %d C * 1000\n", (int) (T_Samp * 1000));)
				DEBUG_PRINT(UARTprintf("Tests performed on this sensor \t %u \n", No_of_tests);)
#endif	// PRINT_UART
			}


			TestValveDrift();
#ifdef LOOSE_VALVE
			TurnValveToStore(V_STORE);
#endif

			// Save error data after all other data has been written to memory
			if((gui32Error & ERRORS_TO_FILTER_TEST) == 0)	// If none of the app filtering error occurred turn off the app filter error
				gui32Error &= ~(ROAM_RESET | APP_FILTER_ERROR);	// Turn off ROAM_RESET flag before saving error code
			else
				gui32Error &= ~(ROAM_RESET);	// Turn off ROAM_RESET flag before saving error code
			MemoryWrite(Test_page, OFFSET_TEST_ERROR, 4, (uint8_t *) &gui32Error);

			DEBUG_PRINT(UARTprintf("Error Code: 0x%x\n", gui32Error);)
			PrintErrors(gui32Error, 1, STATE_MEASUREMENT);
			DEBUG_PRINT(UARTprintf("\n");)
			update_Error();

			update_Test(Test_Number);

			// Pause at end to wait for user to empty waste chamber
			update_Status(STATUS_TEST, OPERATION_TEST_EMPTY_WASTE);
			DEBUG_PRINT(UARTprintf("Empty waste chamber! Press button to continue. \n");)

			PrintTime();

#ifdef TESTING_MODE

			// TODO: Work out the QC stuff
			// Set the standard methods based on the QC solution
			uint8_t qc_passed = 1;
			if(g_QCSolution >= 1 && g_QCSolution <= 5)
			{
				float pH_SM, Ca_SM, TH_SM, NH4_SM, FCl_SM, TCl_SM, Alk_SM, Cond_SM;

				if(g_QCSolution == 1)
				{
					pH_SM = 6.49;
					Ca_SM = 1000;
					TH_SM = 1400;
					NH4_SM = 5;
					FCl_SM = 0;
					TCl_SM = 0;
					Alk_SM = 44;
					Cond_SM = 3230;
				}
				else if(g_QCSolution == 2)
				{
					pH_SM = 7;
					Ca_SM = 500;
					TH_SM = 800;
					NH4_SM = 0;
					FCl_SM = 0.5;
					TCl_SM = 0.5;
					Alk_SM = 300;
					Cond_SM = 2410;
				}
				else if(g_QCSolution == 3)
				{
					pH_SM = 7.5;
					Ca_SM = 150;
					TH_SM = 180;
					NH4_SM = 0;
					FCl_SM = 5;
					TCl_SM = 5;
					Alk_SM = 100;
					Cond_SM = 941;
				}
				else if(g_QCSolution == 4)
				{
					pH_SM = 8.27;
					Ca_SM = 20;
					TH_SM = 40;
					NH4_SM = 0.1;
					FCl_SM = 0;
					TCl_SM = 2;
					Alk_SM = 20;
					Cond_SM = 666;
				}
				else if(g_QCSolution == 5)
				{
					pH_SM = 9.49;
					Ca_SM = 10;
					TH_SM = 25;
					NH4_SM = 0;
					FCl_SM = 0.2;
					TCl_SM = 0.2;
					Alk_SM = 200;
					Cond_SM = 345;
				}

				DEBUG_PRINT(UARTprintf("\n");)

				for(i = 0; i < ISEs.pH_Cr.size; i++)
				{
					float pH_Cr_Samp_25 = Calc_pH_TCor(pH_Cr_Samp[i], 25, T_Therm, K_T_pH_Samp_Sq, K_T_pH_Samp_Ln);
					if(abs_val(pH_Cr_Samp_25 - pH_SM) > 0.1)
					{
						qc_passed = 0;
						DEBUG_PRINT(UARTprintf("pH spot %d out of bounds! Expected: %d, Measured: %d\n", i + 1, (int) (pH_SM * 1000), (int) (pH_Cr_Samp_25 * 1000));)
					}
				}
				for(i = 0; i < ISEs.TH.size; i++)
				{
					float TH_Bound = TH_SM * .1;
					if(TH_Bound < 10)
					{
						TH_Bound = 10;
					}

					if(abs_val(TH_corr[i] - TH_SM) > TH_Bound)
					{
						qc_passed = 0;
						DEBUG_PRINT(UARTprintf("TH spot %d out of bounds! Expected: %d, Measured: %d\n", i + 1, (int) (TH_SM * 1000), (int) (TH_corr[i] * 1000));)
					}
				}
				for(i = 0; i < ISEs.NH4.size; i++)
				{
					float NH4_Bound = NH4_SM * .1;
					if(NH4_Bound < .1)
					{
						NH4_Bound = .1;
					}

					if(abs_val(NH4_NH3_N_Free[i] - NH4_SM) > NH4_Bound)
					{
						qc_passed = 0;
						DEBUG_PRINT(UARTprintf("NH4 spot %d out of bounds! Expected: %d, Measured: %d\n", i + 1, (int) (NH4_SM * 1000), (int) (NH4_NH3_N_Free[i] * 1000));)
					}
				}
				for(i = 0; i < ISEs.Ca.size; i++)
				{
					float Ca_Bound = Ca_SM * .1;
					if(Ca_Bound < 10)
					{
						Ca_Bound = 10;
					}

					if(abs_val(Ca_Hardness[i] - Ca_SM) > Ca_Bound)
					{
						qc_passed = 0;
						DEBUG_PRINT(UARTprintf("Ca spot %d out of bounds! Expected: %d, Measured: %d\n", i + 1, (int) (Ca_SM * 1000), (int) (Ca_Hardness[i] * 1000));)
					}
				}

				if(MEASURE_ALKALINITY)
				{
					if(Alk_Samp[T_Chosen_Alk] != Alk_Samp[T_Chosen_Alk])
					{
						qc_passed = 0;
						DEBUG_PRINT(UARTprintf("Alk mixing failed!\n");)
					}
					else
					{
						float Alk_Bound = Alk_SM * .1;
						if(Alk_Bound < 10)
						{
							Alk_Bound = 10;
						}

						for(i = 0; i < ISEs.pH_H2.size; i++)
						{
							if(abs_val(Alk_Samp[i] - Alk_SM) > Alk_Bound)
							{
								qc_passed = 0;
								DEBUG_PRINT(UARTprintf("Alk spot %d out of bounds! Expected: %d, Measured: %d\n", i + 1, (int) (Alk_SM * 1000), (int) (Alk_Samp[i] * 1000));)
							}
						}
					}
				}

				float Cond_Bound = Cond_SM * .1;
				if(Cond_Bound < 20)
				{
					Cond_Bound = 20;
				}

				if(abs_val(Conductivity - Cond_SM) > Cond_Bound)
				{
					qc_passed = 0;
					DEBUG_PRINT(UARTprintf("Conductivity out of bounds! Expected: %d, Measured: %d\n", (int) (Cond_SM * 1000), (int) (Conductivity * 1000));)
				}

				if(gui32Error & CL_CLEANING_OUT_OF_RANGE != 0)
				{
					qc_passed = 0;
					DEBUG_PRINT(UARTprintf("Cl Cleaning failed preventing FCl and TCl from running!\n");)
				}
				else
				{
					if(MEASURE_FCL)
					{
						if(Cl_FCl_ppm != Cl_FCl_ppm)
						{
							qc_passed = 0;
							DEBUG_PRINT(UARTprintf("FCl mixing failed!\n");)
						}
						else
						{
							float FCl_Bound = FCl_SM * .1;
							if(FCl_Bound < .1)
							{
								FCl_Bound = .1;
							}

							if(abs_val(Cl_FCl_ppm - FCl_SM) > FCl_Bound)
							{
								qc_passed = 0;
								DEBUG_PRINT(UARTprintf("FCl out of bounds! Expected: %d, Measured: %d\n", (int) (FCl_SM * 1000), (int) (Cl_FCl_ppm * 1000));)
							}
						}
					}

					if(MEASURE_TCL)
					{
						if(Cl_TCl_ppm != Cl_TCl_ppm)
						{
							qc_passed = 0;
							DEBUG_PRINT(UARTprintf("TCl mixing failed!\n");)
						}
						else
						{
							float TCl_Bound = TCl_SM * .1;
							if(TCl_Bound < .1)
							{
								TCl_Bound = .1;
							}

							if(abs_val(Cl_TCl_ppm - TCl_SM) > TCl_Bound)
							{
								qc_passed = 0;
								DEBUG_PRINT(UARTprintf("TCl out of bounds! Expected: %d, Measured: %d\n", (int) (TCl_SM * 1000), (int) (Cl_TCl_ppm * 1000));)
							}
						}
					}
				}


				SetLED(GREEN_BUTTON_BLINK, 0);
				if((gui32Error & FAILED_TEST) != 0 || qc_passed == 0)
				{
					SetLED(RED_BUTTON | RED_BUTTON_V, 1);
				}
				else
				{
					SetLED(GREEN_BUTTON | GREEN_BUTTON_V, 1);
					DEBUG_PRINT(UARTprintf("QC PASSED\n\n");)
				}
			}
			else
			{
				SetLED(GREEN_BUTTON_BLINK, 0);
				if((gui32Error & FAILED_TEST) == 0 || DEMO_UNIT == 1)
				{
					SetLED(GREEN_BUTTON | GREEN_BUTTON_V, 1);
				}
				else
				{
					SetLED(RED_BUTTON | RED_BUTTON_V, 1);
				}
			}

			g_QCSolution = 0;

#else
			SetLED(GREEN_BUTTON_BLINK, 0);
			if((gui32Error & FAILED_TEST) == 0 || DEMO_UNIT == 1)
			{
				SetLED(GREEN_BUTTON | GREEN_BUTTON_V, 1);
			}
			else
			{
				SetLED(RED_BUTTON | RED_BUTTON_V, 1);
			}
#endif

#ifdef TESTING_MODE
			uint64_t valve_clock = TimerValueGet64(WTIMER0_BASE);
			DEBUG_PRINT(UARTprintf("Time to valve: %d\n", (uint32_t) ((valve_clock - store_clock) / SysCtlClockGet()));)
			DEBUG_PRINT(UARTprintf("Total time elapsed: %d s, or %d m %d s\n", (uint32_t) ((valve_clock - start_clock)/SysCtlClockGet()), (uint32_t) ((valve_clock - start_clock)/SysCtlClockGet())/60,(uint32_t) ((valve_clock - start_clock)/SysCtlClockGet())%60);)

			TimerDisable(WTIMER0_BASE, TIMER_A);
			TimerConfigure(WTIMER0_BASE, TIMER_CFG_ONE_SHOT);
			TimerIntEnable(WTIMER0_BASE, TIMER_TIMA_TIMEOUT);
#endif

			counter = 0;
			while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == IO_BUTTON_PIN && counter < TIMEOUT)
			{
				SysCtlDelay(SysCtlClockGet()/3000);
				counter++;

				// Break out of loop if another start test command is received
				if(g_ui32DataRx0[0] == CONTINUE_TEST && g_ulSSI0RXTO > 0)
				{
					g_ulSSI0RXTO = 0;
					break;
				}

				// Check if state changed, this happens when abort command is received
				if(g_state != STATE_MEASUREMENT)
					break;
			}

//#ifndef TESTING_MODE
//			if((gui32Error & FAILED_TEST) == 0 || DEMO_UNIT == 1)
//#else
//			if(1)
//#endif
			if((gui32Error & FAILED_TEST) == 0 || DEMO_UNIT == 1)
			{
				update_Status(STATUS_TEST, OPERATION_TEST_COMPLETE);
				DEBUG_PRINT(UARTprintf("Test completed! Error Code: 0x%x\n", gui32Error);)
			}
			else
			{
				update_Status(STATUS_TEST, OPERATION_TEST_FAILED);
				DEBUG_PRINT(UARTprintf("Test failed! Error Code: 0x%x\n", gui32Error);)
			}

			//			update_Status(STATUS_TEST, OPERATION_TEST_COMPLETE);
			while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == 0x00);

			counter = 0;
			while(counter < (TIMEOUT * 5) && GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == IO_BUTTON_PIN)
			{
				SysCtlDelay(SysCtlClockGet()/3000);
				counter++;

				// Break out of loop if continue test command is received
				if(g_ui32DataRx0[0] == CONTINUE_TEST && g_ulSSI0RXTO > 0)
				{
					g_ulSSI0RXTO = 0;
					break;
				}

				// Check if state changed, this happens when abort command is received
				if(g_state != STATE_MEASUREMENT)
					break;
			}
			while(GPIOPinRead(IO_BUTTON_BASE, IO_BUTTON_PIN) == 0);

			g_state = STATE_IDLE;
			break;
		}

		case STATE_UPDATING:
		{
			// Turn off interrupts during OAD
			TimerDisable(TIMER1_BASE, TIMER_A);
			TimerDisable(TIMER2_BASE, TIMER_A);
			TimerDisable(WTIMER1_BASE, TIMER_A);
			if(gBoard >= V6 && HibernateIsActive())
			{
				TimerDisable(WTIMER0_BASE, TIMER_A);
			}
			IntDisable(INT_BATTERY_BASE);

			DEBUG_PRINT(UARTprintf("Waiting for BT to download image!\n");)

			while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == 0)
			{
				// Poll for SSI interuppt and firmware update instruction from BT
				if(g_ui32DataRx0[0] == UPDATE_TIVA_FIRMWARE && g_ulSSI0RXTO > 0)
				{
					g_ulSSI0RXTO = 0; // Reset SSI RX interrupt flag

					JumpToBootLoader();
				} // if SPI firmware update instruction
			}
			while(GPIOPinRead(IO_I2C_USED_BY_BT_BASE, IO_I2C_USED_BY_BT_PIN) == IO_I2C_USED_BY_BT_PIN);

			DEBUG_PRINT(UARTprintf("BT downloaded image and reset!\n");)

			// Pulse BT chip and make sure it responds, don't load anything into SSI buffer in case there is already something there it will be cleared out
			int counter = 0;
			uint8_t Command_received = 0;
			uint8_t attempts = 0;
			while(Command_received == 0 && attempts < 3)
			{
				counter = 0;
				GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, IO_TIVA_RQST_PIN); // I2C used by TIVA
				SysCtlDelay(2000);
				GPIOPinWrite(IO_TIVA_RQST_BASE, IO_TIVA_RQST_PIN, 0x00); // I2C used by TIVA

				while(g_ulSSI0RXTO == 0 && counter < 10000)	// Wait for BT to reply or 1 second to pass
				{
					SysCtlDelay(SysCtlClockGet()/30000);	// Delay 100 us
					counter++;
				}
				if(g_ulSSI0RXTO > 0 && g_ui32DataRx0[0] == REQUEST_TIVA_INSTRUCTION)
					Command_received = 1;

				attempts++;
			}

			if(Command_received == 1)
			{
				InitAnalog();
				userDelay(10, 0);

				update_Dev_Info();	// update device info must be first command sent to BT to get bluetooth services up and running
				// Update cartridge status even when waking from hibernate just in case cartridge was plugged in while in hibernate
				update_Cartridge_Status(gCartridge);
				if(gCartridge == 1)
					update_TCNumbers();

				update_Status(gStatus, gOperation);
				update_MonoCl();
				update_Alkalinity();
				update_Auto_Cal();
				update_Battery(gPumping);
				update_Error();
//				AnalogOff();	// It returns to IDLE and makes sure everything is set up correctly then turns off analog board if necessary
			}

			IntEnable(INT_BATTERY_BASE);

			g_state = STATE_IDLE;
			g_next_state = STATE_IDLE;
			break;
		}
#ifdef TESTING_MODE
		case STATE_FACTORY_CAL:
		{
			// Must send status before updating error so we know where to assign error
			update_Status(STATUS_TEST, OPERATION_TEST_PRECHECK);	// Send status to BT that this is pre-check

			// Set error to 0 at beginning
			gui32Error = 0;
			update_Error();

			// Precheck to make sure everything is good to run calibration
			gui32Error = ROAM_RESET;	// Set global error to ROAM_RESET so its only error at beginning so all errors that appear are during running test
			pui8SysStatus = RequestSystemStatus(); // Get time and user information at beginning of test
			CheckCartridge(pui8SysStatus);
			uint8_t Battery_Percent = BatteryRead(REP_SOC_REG);
			if(Battery_Percent < MIN_BAT_LEVEL && GPIOPinRead(GPIO_PORTE_BASE, GPIO_PIN_0) != 0)	// Check that battery is at minimum level or device is plugged in
			{
				UARTprintf("Battery is too low! Aborting Test!\n");
				gui32Error |= BATTERY_TOO_LOW;
			}

			uint32_t counter = 0;

//			// Check for error that would prevent us from starting calibration, blink light red if this occurs
//			if(gui32Error != ROAM_RESET)	// Something in the cartridge failed (max number of tests exceeded or cartridge expired) or battery check failed
//			{
//				if(ENFORCE_ERRORS != 0)
//				{
//					update_Status(STATUS_TEST, OPERATION_TEST_FAILED);	// Send status to BT that this is pre-check
//
//					gui32Error &= ~ROAM_RESET;	// Remove ROAM_RESET flag since software stopped calibration
//					update_Error();
//
//					SetLED(GREEN_BUTTON | GREEN_BUTTON_V, 0);
//					SetLED(RED_BUTTON | RED_BUTTON_V, 1);
//					while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == GPIO_PIN_3 && counter < TIMEOUT)
//					{
//						SysCtlDelay(SysCtlClockGet()/3000);
//						counter++;
//
//						// Break out of while loop if continue calibration command is received
//						if(g_ui32DataRx0[0] == CONTINUE_TEST && g_ulSSI0RXTO > 0)
//						{
//							g_ulSSI0RXTO = 0;
//							break;
//						}
//
//						// Return to idle if abort command is received
//						if(g_state != STATE_MEASUREMENT)
//						{
//							break;
//						}
//					}
//
//					g_state = STATE_IDLE;
//					g_next_state = STATE_IDLE;
//					break;
//				}
//			}


			if(CheckCalibration(pui8SysStatus, 1) == 1)
				SetLED(GREEN_BUTTON | GREEN_BUTTON_V, 1);
			else
			{
				SetLED(BLUE_BUTTON | BLUE_BUTTON_V, 1);
			}

			struct ISEConfig ISEs;
			FillISEStruct(&ISEs);

			//
			// Checks which parts of calibration to run
			//
			if(ISEs.Config == 0xFF)
			{
				UARTprintf("WARNING! This sensor configuration isn't saved to memory!\n");
				SetLED(GREEN_BUTTON | GREEN_BUTTON_V | BLUE_BUTTON | BLUE_BUTTON_V, 0);
				SetLED(RED_BUTTON, 1);
			}

			uint8_t *pui8TempCoefficients = MemoryRead(PAGE_FACTORY_CAL, OFFSET_TEMP_COEFFICIENT_A, 12);
			float A = Build_float(pui8TempCoefficients);
			float B = Build_float(pui8TempCoefficients + 4);
			float C = Build_float(pui8TempCoefficients + 8);

			if(A != A || B != B || C != C)
			{
				UARTprintf("WARNING! This sensors amperometric temperature sensor hasn't been calibrated!\n");
				SetLED(GREEN_BUTTON | GREEN_BUTTON_V | BLUE_BUTTON | BLUE_BUTTON_V, 0);
				SetLED(RED_BUTTON, 1);
			}

			UARTprintf("\n");

			UARTprintf("What parts of the calibration do you want to run?\n");
			UARTprintf("Type C for conductivity calibration\n");
			UARTprintf("Type F or T for chlorine dummy runs\n");
			UARTprintf("Type R for Thermistor correction\n");
			UARTprintf("Press enter when done, or press button on device! If none of these are typed in everything will be included!\n");

			// Clear UART FIFO
			while(UARTCharsAvail(UART0_BASE))
				UARTCharGetNonBlocking(UART0_BASE);

			uint8_t CONDITION_AMPS = 0;
			uint8_t CALIBRATE_CONDUCTIVITY = 0;
			uint8_t CALIBRATE_THERM = 0;
			uint8_t confirmation = 0;
			while(confirmation == 0)
			{
				if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == 0)
					confirmation = 1;
				if(UARTCharsAvail(UART0_BASE))
				{
					int32_t UART_Rx = UARTCharGet(UART0_BASE);
					UARTCharPutNonBlocking(UART0_BASE, UART_Rx); //echo character
					UARTprintf(", ");

					if(UART_Rx == 0x0D)	// 0x0D = enter; use hex because UART_Rx is defined as int32_t because thats what UARTCharGet returns
						confirmation = 1;
					else if(UART_Rx == 0x43 || UART_Rx == 0x63) // 0x43 = C, 0x63 = c
						CALIBRATE_CONDUCTIVITY = 1;
					else if(UART_Rx == 0x46 || UART_Rx == 0x66) // 0x46 = F, 0x66 = f
						CONDITION_AMPS = 1;
					else if(UART_Rx == 0x54 || UART_Rx == 0x74) // 0x54 = T, 0x74 = t
						CONDITION_AMPS = 1;
					else if(UART_Rx == 0x52 || UART_Rx == 0x72) // 0x52 = R, 0x72 = r
						CALIBRATE_THERM = 1;
				}
			}

			uint8_t approval = 0;
			if(CONDITION_AMPS == 0 && CALIBRATE_CONDUCTIVITY == 0 && CALIBRATE_THERM == 0)
			{
				CONDITION_AMPS = 1;
				CALIBRATE_CONDUCTIVITY = 1;
				CALIBRATE_THERM = 1;
				approval = 1;
			}

			UARTprintf("\nCalibration includes:\n");
			if(CALIBRATE_CONDUCTIVITY)
				UARTprintf("Calibration Conductivity\n");
			if(CONDITION_AMPS)
				UARTprintf("Running Cl Dummy points\n");
			if(CALIBRATE_THERM)
				UARTprintf("Calibrating Thermistor\n");
			UARTprintf("\n");

			if(approval == 0)
			{
				UARTprintf("Is the above list correct?\n");
				UARTprintf("Press enter or push the button on the device to continue\n");
				UARTprintf("Type what needs to be added if incorrect\n");
				while(approval == 0)
				{
					if(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == 0)
						approval = 1;
					if(UARTCharsAvail(UART0_BASE))
					{
						int32_t UART_Rx = UARTCharGet(UART0_BASE);
						UARTCharPutNonBlocking(UART0_BASE, UART_Rx); //echo character
						UARTprintf(", ");

						if(UART_Rx == 0x0D)	// 0x0D = enter; use hex because UART_Rx is defined as int32_t because thats what UARTCharGet returns
							approval = 1;
						else if(UART_Rx == 0x43 || UART_Rx == 0x63) // 0x43 = C, 0x63 = c
							CALIBRATE_CONDUCTIVITY = 1;
						else if(UART_Rx == 0x46 || UART_Rx == 0x66) // 0x46 = F, 0x66 = f
							CONDITION_AMPS = 1;
						else if(UART_Rx == 0x54 || UART_Rx == 0x74) // 0x54 = T, 0x74 = t
							CONDITION_AMPS = 1;
						else if(UART_Rx == 0x52 || UART_Rx == 0x72) // 0x52 = R, 0x72 = r
							CALIBRATE_THERM = 1;
					}
				}
			}

			SetLED(GREEN_BUTTON | GREEN_BUTTON_V | RED_BUTTON | RED_BUTTON_V | BLUE_BUTTON | BLUE_BUTTON_V, 0);
			if(g_state == STATE_FACTORY_CAL)
				SetLED(GREEN_BUTTON_BLINK, 1);

			while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == 0);

			if(counter == (TIMEOUT * 5))
				g_state = STATE_IDLE;

			// Return to idle if abort command is received
			if(g_state != STATE_FACTORY_CAL)
				break;

			update_Status(STATUS_TEST, OPERATION_TEST_RINSE);

			UARTprintf("\n");

			uint8_t FactoryCalCheck = 1;
			uint8_t ThermCheck = 1;
			uint8_t CondCheck = 1;
//			uint8_t ClCheck = 1;

			//			// TODO: Set pump variables for test
			// Variables to control pump and valve
			uint8_t Number_of_bubbles_Cond = 10;// - Rinse_pumped;
//			uint8_t Number_of_bubbles_Prerinse = 3;// - Rinse_pumped;
//			uint8_t Number_of_bubbles_samp = 4;
			uint8_t Number_of_bubbles_Postrinse = 1;

			uint8_t Test_Number = FindTestNumber() + 1; // Add 1 to go to next free spot in memory

			float PumpVol_air_bubble = 33.6;			// Volume uL of air bubble
			uint8_t PumpVol_Large_air_bubble = 84;		// Large air bubble on pre/post rinse
			float PumpVol_Solution = 33.6;				// Volume uL for solution between air bubbles
			float PumpVol_Clean = 117.6;				// Volume for clean plug
			float PumpVol_Clean_center = 125.86;		// Volume to center clean plug

			float PumpVol_Rinse = 100.8;		// Volume of prerinse after air bubbles
//			float PumpVol_Sample = 134.4;		// Volume of sample after bubbles
			float PumpVol_plug = 141.29;		// Volume to center the measurement plugs on the sensors and reference.
//			float PumpVol_plug_samp = 131.37;	// Volume to center sample plug for measurement
			uint16_t PumpVol_sample_rinse = 840;	// How much sample to pull before metering, large to clear air out of chip before metering buffers

			uint16_t PumpVol_Sample_Prime = 403;	// Volume to prime sample tube before initial rinse

			// Variables to control mixing
			uint16_t diffusion_time = 0;				// Time to wait after oscillating in mixing chamber
			uint8_t mix_cycles = 5; //10;				// Number of forward backward pump cycles
			uint16_t Steps_cycles = 1500;// * gPump_Ratio;			// Steps to pump forward/backward

			// Priming B1
			float PumpVol_tube_bubble = 16.8;
			float PumpVol_tube_prime_buffers = 33.6;
//			float PumpVol_tube_prime = 16.8;

			// Read step values from memory
			float PumpVol_Buffer = 13.22;	// Adjust this number for B1
//			float PumpVol_B2 = 13.22;	// Adjust this number for B2

			// Initialize floats to hold pump variables
			float PumpVolRev, Pump_Ratio;

			// Read from Tiva EEPROM the pump specs
			EEPROMRead((uint32_t *) &PumpVolRev, OFFSET_PUMP_VOL_PER_REV, 4);
			EEPROMRead((uint32_t *) &Pump_Ratio, OFFSET_PUMP_DEAD_SPOT, 4);

			if(PumpVolRev != PumpVolRev)
				PumpVolRev = 16.8;
			if(Pump_Ratio != Pump_Ratio)
				Pump_Ratio = 0.61;

//			// Variable to control pumping T1
//			uint16_t Steps_PreT1 = 3000;	// Number of pump steps to pull sample before buffer, leaving this as steps so pump starts metering in a known location
//			float PumpVol_PostT1 = 117.6 - ((Steps_PreT1/1000) * PumpVolRev);	// Volume to pull sample after buffer
//			float PumpVol_follow_T1 = 19.55;	// Volume to get T1 mixture into mixing chamber
//			float PumpVol_center_T1 = 131.65;	// Volume to place mixed sample over sensor

			// Variable to control pumping B1
			uint16_t Steps_PreB1 = 2000;	// Number of pump steps to pull sample before buffer
			float PumpVol_PostB1 = 100.8 - ((Steps_PreB1/1000) * PumpVolRev);	// Number of pump steps to pull sample after buffer
			//			uint16_t Steps_follow_B1 = 600;// * gPump_Ratio;	// Steps to pump sample to get B1 mixture into mixing chamber
			float PumpVol_center = 119.67;		// Steps to pump to place mixed sample over sensor

			// Variables to control pumping C2
//			uint16_t Steps_Precond = 2000;// * gPump_Ratio;		// Number of pump steps to pull sample before buffer
//			uint16_t Steps_Postcond = 4000;// * gPump_Ratio;	// Number of pump steps to pull sample after buffer
			float PumpVol_C2 = 7.27;							// Volume of conditioner, should be constant
			float PumpVol_follow_C2 = 15.28 - PumpVol_C2;// * gPump_Ratio;	// Change this number to change where in the mixing chamber the sample and conditioner are mixed
			float PumpVol_center_B2 = 117.74 + PumpVol_C2;// * gPump_Ratio;		// Steps to pump to place mixed sample over sensor

			// Variables to control mixing B2
			//			uint16_t Steps_sample_between_Cl = 10000 + Steps_follow_B1 + Steps_center;
//			uint16_t PumpVol_sample_between_Cl = 840;
			//			uint16_t Steps_back = (5500 + Steps_C2);// * gPump_Ratio; 		// Steps to pump mixed conditioner/sample back into valve before adding buffer, add C2 steps so pump starts at zero for buffer
			float PumpVol_back = 100.8 + 15.28;	// Volume to pump mixed conditioner/sample back into valve before adding buffer, add C2 and follow steps so pump starts at zero for B2
			float PumpVol_forward = (100.8 + 15.28 - PumpVolRev - PumpVol_Buffer);		// Change this number to change where in mixing chamber the sample and buffer are mixed, pump forward the backward amount less a pump revolution and B2 volume


			// Variables to control pump speed
			// 6000 is slow pumping being used for everything else
			// Slowest speed is 8000, may be able to go slower but the function will need to be changed
			// Fastest speed is 3000, may work up to 2500 depends on the pump
			uint16_t Speed_Fast = 3000;
			uint16_t Speed_Metering = 6000;

//			uint16_t Cond_delay = 1000;

			uint8_t i;

			uint16_t valve_delay = 1000;
			uint16_t valve_delay_after_air = 100; //1000;
			uint16_t valve_delay_metering = 500; // 2000;

			if((gui32Error & 0) != 0)
				break;

			float Therm_corr;
			float Cond_Slope;
			if(CALIBRATE_CONDUCTIVITY || CALIBRATE_THERM)
			{
				FindPossitionOneValve();
				// Prime sample tube before test
				UARTprintf("Fill sample vial with low conductivity calibrant\n");
				if(CALIBRATE_THERM)
					UARTprintf("To calibrate Thermistor measure the sample temperature IN THE SAMPLE VIAL!!!\n");

				BuzzerSound(400);
				SetLED(BLUE_BUTTON_BLINK, 1);
				while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == GPIO_PIN_3);
				while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == 0);
				SetLED(BLUE_BUTTON_BLINK, 0);

				float T_Therm_S = ReadThermistor();

				UARTprintf("Priming sample tube... \n");
				RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
				PumpVolume(FW, PumpVol_Sample_Prime, Speed_Fast, 1);
				userDelay(valve_delay, 1);

				float T_Therm_F = ReadThermistor();
				UARTprintf("Therm Start and Final Temps:\t%d\t%d\tC*1000\n", (int) (T_Therm_S * 1000), (int) (T_Therm_F * 1000));

				if(CALIBRATE_THERM)
				{
					UARTprintf("Type in measured vial temperature in the format: '##.##'\n");

					BuzzerSound(400);
					SetLED(BLUE_BUTTON_BLINK, 1);

					int32_t UART_Rx[4] = {0,0,0,0};
					uint8_t check = 0;
					while(check != 1)
					{
						// Clear UART FIFO
						while(UARTCharsAvail(UART0_BASE))
							UARTCharGetNonBlocking(UART0_BASE);

						uint8_t count = 0;
						for(count = 0; count < 4; count++)
						{
							UART_Rx[count] = UARTCharGet(UART0_BASE);
							UARTCharPutNonBlocking(UART0_BASE, UART_Rx[count]); //echo character

							if(UART_Rx[count] == 0x0D)	// If the user pressed enter
							{
								UARTprintf("\nReceived enter, breaking for loop\n"); // Set this up so the user can press enter to restart typing the number
								break;
							}
						}
						UARTprintf("\n");

						// Check that the 4 characters are a number, followed by decimal, followed by two numbers, also check all 4 characters were recevied
						if((UART_Rx[0] >= 0x30 && UART_Rx[0] <= 0x39) && (UART_Rx[1] >= 0x30 && UART_Rx[1] <= 0x39) && (UART_Rx[2] == 0x2E) && (UART_Rx[3] >= 0x30 && UART_Rx[3] <= 0x39) && count == 4)
							check = 1;
						else
							UARTprintf("Entered data not in required format, needs to be ##.#! Try again!\n");
					}
					SetLED(BLUE_BUTTON_BLINK, 0);

					float T_Vial = (UART_Rx[0] - 0x30) * 10 + (UART_Rx[1] - 0x30) + ((float) (UART_Rx[3] - 0x30))/10;	// Convert string into a float
					UARTprintf("Converted to float: %d * 1000\n\n", (int) (T_Vial * 1000));

					//					UARTprintf("Received characters: %c%c%c%c\n", UART_Rx[0], UART_Rx[1], UART_Rx[2], UART_Rx[3]);

					Therm_corr = (T_Therm_F - T_Therm_S) / (T_Therm_S - T_Vial);
					UARTprintf("Thermistor correction found: %d / 1000\n", (int) (Therm_corr * 1000));
					if(Therm_corr >= -1 && Therm_corr <= -0.5)
					{
						UARTprintf("Thermistor correction seems realistic, saving to cartridge memory!\n");
						MemoryWrite(PAGE_FACTORY_CAL, OFFSET_THERM_CORRECTION, 4, (uint8_t *) &Therm_corr);
					}
					else
					{
						UARTprintf("Thermistor correction outside of expected range, not saving to memory!\n");
						FactoryCalCheck = 0;
						ThermCheck = 0;
					}
				}

				//
				// Conductivity low point calibrant
				//
				if((gui32Error & 0) == 0)
				{
					update_Status(STATUS_TEST, OPERATION_TEST_RINSE);
					UARTprintf("Pumping low point conductivity calibrant... \n");

					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
					FindPossitionZeroPump();
					for (i = 0; i < Number_of_bubbles_Cond; i++) // Loop over air/solution cycle 3 times for single solution
					{
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 1);
						if(i == (Number_of_bubbles_Cond - 1))
							PumpVolume(FW, PumpVol_Large_air_bubble, Speed_Fast, 1);
						userDelay(valve_delay, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
						if(i == 0 && BUBBLES_IN_TUBE)
							PumpVolume(FW, PumpVol_tube_bubble, Speed_Fast, 1);
						PumpVolume(FW, PumpVol_Solution, Speed_Fast, 1);
						if(i != (Number_of_bubbles_Cond - 1))
							userDelay(valve_delay, 1);
					}
					PumpVolume(FW, PumpVol_Rinse, Speed_Fast, 1);
					userDelay(valve_delay, 1);
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					PumpVolume(FW, PumpVol_plug, Speed_Fast, 1);
				}

				float CalConductivity[3] = {0,0,0};
				float CalConductivity_alt[3] = {0,0,0};
				float T_Cond[3] = {0,0,0};
				float Conductivity_cal[3] = {64.1, 64.1, 64.1};
//				float Conductivity_cal[3] = {364, 364, 364};

				// Measure conductivity
				// Set RE and CE floating and close RE/CE loop for conductivity
				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);

				// Set low current range
				// 10.7 uApp R = 309k + 499k = 808k
				IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
				IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);

//				// Set mid current range
//				// 20 uApp R = 430k
//				IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 1);
//				IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

				uint8_t ExtraCondPoints = 3;
				for(i = 0; i < 3; i++)
				{
					if(i > 0 && ExtraCondPoints > 0)
					{
						i = 0;
						ExtraCondPoints--;
					}

					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
					PumpVolume(FW, PumpVol_Rinse + PumpVol_Solution, Speed_Fast, 1);
					userDelay(valve_delay, 1);
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					PumpVolume(FW, PumpVol_plug, Speed_Fast, 1);

					ConnectMemory(0);

					// Set low current range
					// 10.7 uApp R = 309k + 499k = 808k
					IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
					IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);

					if(gABoard >= ARV1_0B)
					{
						InitWaveGen(0, 1000);	// Change frequency to 1kHz

						// Set low current range
						// 10.7 uApp R = 309k + 499k = 808k
						IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
						IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);

						uint8_t Check = 0, attempt = 0;

						while(Check != 1)
						{
							WaveGenSet(1);

							Check = CheckCond(1000);
							if(attempt == 5)
							{
								gui32Error |= WAVE_GEN_FAIL;
								break;
							}

							if(Check != 1)
							{
								InitWaveGen(1, 1000);
								attempt++;
							}
						}

						CalConductivity_alt[i] = ConductivityMovingAvg(1000);		// uV

						UARTprintf("Cond Raw 1kHz: %d\n", (int) (CalConductivity_alt[i] * 1000));

						WaveGenSet(0);

						InitWaveGen(0, 5000);	// Change frequency back to 5kHz
					}

					// Set low current range
					// 10.7 uApp R = 309k + 499k = 808k
					IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
					IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);

					uint8_t Check = 0;
					while(Check != 1)
					{
						WaveGenSet(1);

						Check = CheckCond(COND_FREQ);
						if(Check != 1)
						{
							InitWaveGen(1, COND_FREQ);
						}
					}

					CalConductivity[i] = ConductivityMovingAvg(COND_FREQ);
					UARTprintf("Low Cond Raw: %d\n", (int) (CalConductivity[i] * 1000));

					WaveGenSet(0);	// Turn off waveform generator when switching ranges



					ConnectMemory(1);

					T_Cond[i] = MeasureTemperature(1);
					// Perform temperature correction here after calculations for ISEs so we are using the conductivity at temperature, not the adjusted conductivity
					if(ExtraCondPoints == 0)
						Conductivity_cal[i] *= (1 + COND_TCOMP_CONDLOW*(T_Cond[i] - 25));	// Multiply because we already have cond at 25 and want it at measured temperature
					UARTprintf("Temperature: %d C * 1000\n", (int) (T_Cond[i] * 1000));
				}

				// Push air back into cond cal port before moving to next solution
				if(BUBBLES_IN_TUBE)
				{
					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
					PumpVolume(BW, PumpVol_tube_bubble, Speed_Fast, 1);
					userDelay(valve_delay, 1);
				}

				if(PURGE_SAMPLE)
				{
					UARTprintf("Purging sample tube by pushing air backwards into it\n");
					RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
					PumpVolume(FW, PumpVol_sample_rinse + PumpVol_Sample_Prime + 33.6, Speed_Fast, 0);
					userDelay(valve_delay_after_air, 0);
					RunValveToPossition_Bidirectional(V_SAMP, VALVE_STEPS_PER_POSITION);
					PumpVolume(BW, PumpVol_Sample_Prime, Speed_Fast, 0);
					userDelay(valve_delay, 0);
				}

				//
				// Conductivity mid point calibrant
				//
				UARTprintf("Fill sample vial with mid conductivity calibrant\n");

				BuzzerSound(400);
				SetLED(BLUE_BUTTON_BLINK, 1);
				while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == GPIO_PIN_3);
				while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == 0);
				SetLED(BLUE_BUTTON_BLINK, 0);

				if((gui32Error & 0) == 0)
				{
					update_Status(STATUS_TEST, OPERATION_TEST_RINSE);
					UARTprintf("Pumping mid point conductivity calibrant... \n");

					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
					FindPossitionZeroPump();
					for (i = 0; i < Number_of_bubbles_Cond; i++) // Loop over air/solution cycle 3 times for single solution
					{
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 1);
						if(i == (Number_of_bubbles_Cond - 1))
							PumpVolume(FW, PumpVol_Large_air_bubble, Speed_Fast, 1);
						userDelay(valve_delay, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
						if(i == 0 && BUBBLES_IN_TUBE)
							PumpVolume(FW, PumpVol_tube_bubble, Speed_Fast, 1);
						PumpVolume(FW, PumpVol_Solution, Speed_Fast, 1);
						if(i != (Number_of_bubbles_Cond - 1))
							userDelay(valve_delay, 1);
					}
					PumpVolume(FW, PumpVol_Rinse, Speed_Fast, 1);
					userDelay(valve_delay, 1);
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					PumpVolume(FW, PumpVol_plug, Speed_Fast, 1);
				}

				float Mid_CalConductivity[3] = {0,0,0};
				float Mid_CalConductivity_alt[3] = {0,0,0};
				float Mid_T_Cond[3] = {0,0,0};
				float Mid_Conductivity_cal[3] = {1000, 1000, 1000};	// {1077, 1077, 1077};

				// Measure conductivity
				// Set RE and CE floating and close RE/CE loop for conductivity
				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);

				// Set mid current range
				// 20 uApp R = 430k
				IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 1);
				IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

				ExtraCondPoints = 3;
				for(i = 0; i < 3; i++)
				{
					if(i > 0 && ExtraCondPoints > 0)
					{
						i = 0;
						ExtraCondPoints--;
					}

					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
					PumpVolume(FW, PumpVol_Rinse + PumpVol_Solution, Speed_Fast, 1);
					userDelay(valve_delay, 1);
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					PumpVolume(FW, PumpVol_plug, Speed_Fast, 1);

					ConnectMemory(0);

					// Set mid current range
					// 20 uApp R = 430k
					IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 1);
					IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

					if(gABoard >= ARV1_0B)
					{
						InitWaveGen(0, 1000);	// Change frequency to 1kHz

						// Set mid current range
						// 20 uApp R = 430k
						IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 1);
						IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

						uint8_t Check = 0, attempt = 0;

						while(Check != 1)
						{
							WaveGenSet(1);

							Check = CheckCond(1000);
							if(attempt == 5)
							{
								gui32Error |= WAVE_GEN_FAIL;
								break;
							}

							if(Check != 1)
							{
								InitWaveGen(1, 1000);
								attempt++;
							}
						}

						Mid_CalConductivity_alt[i] = ConductivityMovingAvg(1000);		// uV

						UARTprintf("Cond Raw 1kHz: %d\n", (int) (Mid_CalConductivity_alt[i] * 1000));

						WaveGenSet(0);

						InitWaveGen(0, 5000);	// Change frequency back to 5kHz
					}

					// Set mid current range
					// 20 uApp R = 430k
					IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 1);
					IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

					uint8_t Check = 0;
					while(Check != 1)
					{
						WaveGenSet(1);

						Check = CheckCond(COND_FREQ);
						if(Check != 1)
						{
							InitWaveGen(1, COND_FREQ);
						}
					}

					Mid_CalConductivity[i] = ConductivityMovingAvg(COND_FREQ);
					UARTprintf("Cond Raw: %d\n", (int) (Mid_CalConductivity[i] * 1000));

					WaveGenSet(0);	// Turn off waveform generator when switching ranges

					ConnectMemory(1);

					Mid_T_Cond[i] = MeasureTemperature(1);

					if(ExtraCondPoints == 0)
						Mid_Conductivity_cal[i] *= (1 + 0.02 *(Mid_T_Cond[i] - 25));	// Multiply because we already have cond at 25 and want it at measured temperature
					UARTprintf("Temperature: %d C * 1000\n", (int) (Mid_T_Cond[i] * 1000));
				}

				//
				// Conductivity high point calibrant
				//
				UARTprintf("Fill sample vial with high conductivity calibrant\n");

				BuzzerSound(400);
				SetLED(BLUE_BUTTON_BLINK, 1);
				while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == GPIO_PIN_3);
				while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == 0);
				SetLED(BLUE_BUTTON_BLINK, 0);

				if((gui32Error & 0) == 0)
				{
					update_Status(STATUS_TEST, OPERATION_TEST_RINSE);
					UARTprintf("Pumping high point conductivity calibrant... \n");

					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
					FindPossitionZeroPump();
					for (i = 0; i < Number_of_bubbles_Cond; i++) // Loop over air/solution cycle 3 times for single solution
					{
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 1);
						if(i == (Number_of_bubbles_Cond - 1))
							PumpVolume(FW, PumpVol_Large_air_bubble, Speed_Fast, 1);
						userDelay(valve_delay, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
						if(i == 0 && BUBBLES_IN_TUBE)
							PumpVolume(FW, PumpVol_tube_bubble, Speed_Fast, 1);
						PumpVolume(FW, PumpVol_Solution, Speed_Fast, 1);
						if(i != (Number_of_bubbles_Cond - 1))
							userDelay(valve_delay, 1);
					}
					PumpVolume(FW, PumpVol_Rinse, Speed_Fast, 1);
					userDelay(valve_delay, 1);
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					PumpVolume(FW, PumpVol_plug, Speed_Fast, 1);
				}

				float High_CalConductivity[3] = {0,0,0};
				float High_CalConductivity_alt[3] = {0,0,0};
				float High_T_Cond[3] = {0,0,0};
				float High_Conductivity_cal[3] = {2000, 2000, 2000};

				// Measure conductivity
				// Set RE and CE floating and close RE/CE loop for conductivity
				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
				IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);

				// Set high current range
				// 45 uApp R = 180k
				IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
				IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

				ExtraCondPoints = 3;
				for(i = 0; i < 3; i++)
				{
					if(i > 0 && ExtraCondPoints > 0)
					{
						i = 0;
						ExtraCondPoints--;
					}

					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
					PumpVolume(FW, PumpVol_Rinse + PumpVol_Solution, Speed_Fast, 1);
					userDelay(valve_delay, 1);
					RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
					PumpVolume(FW, PumpVol_plug, Speed_Fast, 1);

					ConnectMemory(0);

					if(gABoard >= ARV1_0B)
					{
						InitWaveGen(0, 1000);	// Change frequency to 1kHz

						// Set high current range
						// 45 uApp R = 180k
						IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
						IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

						uint8_t Check = 0, attempt = 0;

						while(Check != 1)
						{
							WaveGenSet(1);

							Check = CheckCond(1000);
							if(attempt == 5)
							{
								gui32Error |= WAVE_GEN_FAIL;
								break;
							}

							if(Check != 1)
							{
								InitWaveGen(1, 1000);
								attempt++;
							}
						}

						High_CalConductivity_alt[i] = ConductivityMovingAvg(1000);		// uV

						UARTprintf("Cond Raw 1kHz: %d\n", (int) (High_CalConductivity_alt[i] * 1000));

						WaveGenSet(0);

						InitWaveGen(0, 5000);	// Change frequency back to 5kHz
					}

					// Set high current range
					// 45 uApp R = 180k
					IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
					IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 1);

					uint8_t Check = 0;
					while(Check != 1)
					{
						WaveGenSet(1);

						Check = CheckCond(COND_FREQ);
						if(Check != 1)
						{
							InitWaveGen(1, COND_FREQ);
						}
					}

					High_CalConductivity[i] = ConductivityMovingAvg(COND_FREQ);
					UARTprintf("Cond Raw: %d\n", (int) (High_CalConductivity[i] * 1000));

					WaveGenSet(0);	// Turn off waveform generator when switching ranges

					ConnectMemory(1);

					High_T_Cond[i] = MeasureTemperature(1);
					// Perform temperature correction here after calculations for ISEs so we are using the conductivity at temperature, not the adjusted conductivity
					if(ExtraCondPoints == 0)
						High_Conductivity_cal[i] *= (1 + 0.02 *(High_T_Cond[i] - 25));	// Multiply because we already have cond at 25 and want it at measured temperature
					UARTprintf("Temperature: %d C * 1000\n", (int) (High_T_Cond[i] * 1000));
				}

				float Conductivity_low_cal = (Conductivity_cal[0] + Conductivity_cal[1] + Conductivity_cal[2]) / 3.0;
				float Conductivity_reading_low = (CalConductivity[0] + CalConductivity[1] + CalConductivity[2]) / 3.0;
				float Conductivity_reading_low_alt = (CalConductivity_alt[0] + CalConductivity_alt[1] + CalConductivity_alt[2]) / 3.0;

				float Conductivity_mid_cal = (Mid_Conductivity_cal[0] + Mid_Conductivity_cal[1] + Mid_Conductivity_cal[2]) / 3.0;
				float Conductivity_reading_mid = (Mid_CalConductivity[0] + Mid_CalConductivity[1] + Mid_CalConductivity[2]) / 3.0;
				float Conductivity_reading_mid_alt = (Mid_CalConductivity_alt[0] + Mid_CalConductivity_alt[1] + Mid_CalConductivity_alt[2]) / 3.0;

				float Conductivity_high_cal = (High_Conductivity_cal[0] + High_Conductivity_cal[1] + High_Conductivity_cal[2]) / 3.0;
				float Conductivity_reading_high = (High_CalConductivity[0] + High_CalConductivity[1] + High_CalConductivity[2]) / 3.0;
				float Conductivity_reading_high_alt = (High_CalConductivity_alt[0] + High_CalConductivity_alt[1] + High_CalConductivity_alt[2]) / 3.0;

				float Low_Current, Mid_Current, High_Current;
				EEPROMRead((uint32_t *) &Low_Current, OFFSET_COND_I_LOW, 4);
				EEPROMRead((uint32_t *) &Mid_Current, OFFSET_COND_I_MID, 4);
				EEPROMRead((uint32_t *) &High_Current, OFFSET_COND_I_HIGH, 4);

				if(Low_Current != Low_Current)
					Low_Current = 10.76 * 0.795;	// Average from circuits before ARV1_0B
				if(Mid_Current != Mid_Current)
					Mid_Current = 19.89 * 0.8;	// Average from circuits before ARV1_0B
				if(High_Current != High_Current)
					High_Current = 43.57 * .812;	// Average from circuits before ARV1_0B

				float Low_Current_alt, Mid_Current_alt, High_Current_alt;
				EEPROMRead((uint32_t *) &Low_Current_alt, OFFSET_COND_ALT_I_LOW, 4);
				EEPROMRead((uint32_t *) &Mid_Current_alt, OFFSET_COND_ALT_I_MID, 4);
				EEPROMRead((uint32_t *) &High_Current_alt, OFFSET_COND_ALT_I_HIGH, 4);

				if(Low_Current_alt != Low_Current_alt)
					Low_Current_alt = Low_Current * 1.22;	// Because of low pass filter the frequency at 1kHz will be ~22% higher than 5kHz
				if(Mid_Current_alt != Mid_Current_alt)
					Mid_Current_alt = Mid_Current * 1.22;	// Because of low pass filter the frequency at 1kHz will be ~22% higher than 5kHz
				if(High_Current_alt != High_Current_alt)
					High_Current_alt = High_Current * 1.22;	// Because of low pass filter the frequency at 1kHz will be ~22% higher than 5kHz

				Cond_Slope = (High_Current*1000000/Conductivity_reading_high - Mid_Current*1000000/Conductivity_reading_mid)/((Conductivity_high_cal - Conductivity_mid_cal));
				float Cond_Slope_alt;
				if(gABoard >= ARV1_0B)
					Cond_Slope_alt = ((1000000 * High_Current_alt)/Conductivity_reading_high_alt - (1000000 * Mid_Current_alt)/Conductivity_reading_mid_alt)/((Conductivity_high_cal - Conductivity_mid_cal));

				UARTprintf("Low calibrant:\n");
				UARTprintf("Temperature Corrected conductivity: %d, %d, %d uS/cm * 1000\n", (int) (Conductivity_cal[0] * 1000), (int) (Conductivity_cal[1] * 1000), (int) (Conductivity_cal[2] * 1000));
				UARTprintf("Average temperature corrected conductivity: %d uS/cm * 1000\n", (int) (Conductivity_low_cal * 1000));
				UARTprintf("Average conductivity reading: %d\n", (int) (Conductivity_reading_low * 1000));


				Conductivity_reading_low = Low_Current / Conductivity_reading_low;	// Current adjust this reading so the factory calibration holds across Roam units
				if(gABoard >= ARV1_0B)
					UARTprintf("1000 Hz Current adjusted conductivity reading: %d\n", (int) (Low_Current_alt * 1000000 / Conductivity_reading_low_alt * 1000));
				UARTprintf("%d Hz Current adjusted conductivity reading: %d\n", COND_FREQ, (int) (Conductivity_reading_low * 1000000 * 1000));

				UARTprintf("\nMid calibrant:\n");
				UARTprintf("Temperature Corrected conductivity: %d, %d, %d uS/cm * 1000\n", (int) (Mid_Conductivity_cal[0] * 1000), (int) (Mid_Conductivity_cal[1] * 1000), (int) (Mid_Conductivity_cal[2] * 1000));
				UARTprintf("Average temperature corrected conductivity: %d uS/cm * 1000\n", (int) (Conductivity_mid_cal * 1000));
				UARTprintf("Average conductivity reading: %d\n", (int) (Conductivity_reading_mid * 1000));

				if(gABoard >= ARV1_0B)
					UARTprintf("1000 Hz Current adjusted conductivity reading: %d\n", (int) (Mid_Current_alt * 1000000 / Conductivity_reading_mid_alt * 1000));
				UARTprintf("%d Hz Current adjusted conductivity reading: %d\n", COND_FREQ, (int) (Mid_Current * 1000000 / Conductivity_reading_mid * 1000));

				UARTprintf("\nHigh calibrant:\n");
				UARTprintf("Temperature Corrected conductivity: %d, %d, %d uS/cm * 1000\n", (int) (High_Conductivity_cal[0] * 1000), (int) (High_Conductivity_cal[1] * 1000), (int) (High_Conductivity_cal[2] * 1000));
				UARTprintf("Average temperature corrected conductivity: %d uS/cm * 1000\n", (int) (Conductivity_high_cal * 1000));
				UARTprintf("Average conductivity reading: %d\n", (int) (Conductivity_reading_high * 1000));

				if(gABoard >= ARV1_0B)
					UARTprintf("1000 Hz Current adjusted conductivity reading: %d\n", (int) (High_Current_alt * 1000000 / Conductivity_reading_high_alt * 1000));
				UARTprintf("%d Hz Current adjusted conductivity reading: %d\n", COND_FREQ, (int) (High_Current * 1000000 / Conductivity_reading_high * 1000));

				if(gABoard >= ARV1_0B)
					UARTprintf("\n1000 Hz Factory Conductivity Slope:\t%d\n", (int) (Cond_Slope_alt * 1000));
				UARTprintf("%d Hz Factory Conductivity Slope:\t%d\n", COND_FREQ, (int) (Cond_Slope * 1000));

				if(Cond_Slope > 0.1 && Cond_Slope < 0.3)
				{
					UARTprintf("\nConductivity Factory Cal check passed, saving to memory\n");
					MemoryWrite(PAGE_FACTORY_CAL, OFFSET_COND_LOW_POINT_CAL, 4, (uint8_t *) &Conductivity_low_cal);
					MemoryWrite(PAGE_FACTORY_CAL, OFFSET_COND_READ_LOW_POINT, 4, (uint8_t *) &Conductivity_reading_low);

					if(gABoard >= ARV1_0B)
					{
						MemoryWrite(PAGE_FACTORY_CAL, OFFSET_FACTORY_COND_SLOPE, 4, (uint8_t *) &Cond_Slope_alt);	// 1 kHz
						MemoryWrite(PAGE_FACTORY_CAL, OFFSET_FACTORY_COND_SLOPE_ALT, 4, (uint8_t *) &Cond_Slope);	// 5 kHz
					}
					else
						MemoryWrite(PAGE_FACTORY_CAL, OFFSET_FACTORY_COND_SLOPE, 4, (uint8_t *) &Cond_Slope);	// 1 kHz
				}
				else
				{
					UARTprintf("Conductivity Factory Cal check failed, NOT saving to memory\n");
					FactoryCalCheck = 0;
					CondCheck = 0;
				}

				// Push air back into cond cal port before moving to next solution
				if(BUBBLES_IN_TUBE)
				{
					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
					PumpVolume(BW, PumpVol_tube_bubble, Speed_Fast, 1);
					userDelay(valve_delay, 1);
				}

				if(PURGE_SAMPLE)
				{
					UARTprintf("Purging sample tube by pushing air backwards into it\n");
					RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
					PumpVolume(FW, PumpVol_sample_rinse + PumpVol_Sample_Prime + 33.6, Speed_Fast, 0);
					userDelay(valve_delay_after_air, 0);
					RunValveToPossition_Bidirectional(V_SAMP, VALVE_STEPS_PER_POSITION);
					PumpVolume(BW, PumpVol_Sample_Prime, Speed_Fast, 0);
					userDelay(valve_delay, 0);
				}
			}

			// Set RE and CE floating and close RE/CE loop
			IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
			IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);

			if(CONDITION_AMPS)
			{
				// Prime sample tube before test
				UARTprintf("Put sample tube in 5 ppm Cl2 and press button\n");

				BuzzerSound(400);
				SetLED(BLUE_BUTTON_BLINK, 1);
				while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == GPIO_PIN_3);
				while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == 0);
				SetLED(BLUE_BUTTON_BLINK, 0);

				UARTprintf("Pumping for 5 minutes to pre-bleach... \n");
				RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
				PumpVolume(FW, PumpVol_Sample_Prime + PumpVol_sample_rinse, Speed_Fast, 1);

				// Slowly pump sample for 5 minutes
				PumpStepperRunTimeSpeed_AbortReady(FW, 150, 6000);
				PumpStepperRunTimeSpeed_AbortReady(FW, 150, 6000);
//				userDelay(300000, 1);

				if(PURGE_SAMPLE)
				{
					UARTprintf("Purging sample tube by pushing air backwards into it\n");
					RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
					PumpVolume(FW, PumpVol_sample_rinse + PumpVol_Sample_Prime + 33.6, Speed_Fast, 0);
					userDelay(valve_delay_after_air, 0);
					RunValveToPossition_Bidirectional(V_SAMP, VALVE_STEPS_PER_POSITION);
					PumpVolume(BW, PumpVol_Sample_Prime, Speed_Fast, 0);
					userDelay(valve_delay, 0);
				}

				TestValveDrift();
			}

			int k;
			// Create variables for Cl mixing
			float Cl_nA_FCl[9] = {0,0,0,0,0,0,0,0,0};
			float Cl_nA_TCl[9] = {0,0,0,0,0,0,0,0,0};
			float T_Samp_B2 = T_assume;
			float T_Samp_B1 = T_assume;
//			float Amp_Voltage_Set = 470;
			float Amp_Voltage_Set = 470 - REF_DRIFT;

//			uint8_t ExtraStartRuns = 2;

			if(CONDITION_AMPS)
				for(k = 0; k < 2; k++)
				{
					// Prime sample tube before test
					UARTprintf("Fill sample vial with 5 ppm chlorine for dummy runs and press button!\n");

					if(k == 1)
					{
						BuzzerSound(400);
						SetLED(BLUE_BUTTON_BLINK, 1);
						while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == GPIO_PIN_3);
						while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == 0);
						SetLED(BLUE_BUTTON_BLINK, 0);
					}

					UARTprintf("Priming sample... \n");
					RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
					PumpVolume(FW, PumpVol_Sample_Prime, Speed_Fast, 1);
					userDelay(valve_delay, 1);

					// Push air back into sample port before moving to next solution
					if(BUBBLES_IN_TUBE)
					{
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
						PumpVolume(BW, PumpVol_tube_bubble, Speed_Fast, 1);
						userDelay(valve_delay, 1);
					}


					//
					// Clean amperometrics
					//
					if((gui32Error & 0) == 0)
					{
						// Pump in rinse over amperometrics for cleaning
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
						FindPossitionZeroPump();
						userDelay(valve_delay_after_air, 1);
						UARTprintf("Pumping cleaning solution to clean amperometrics!\n");
						if(ISEs.Config == PH_CL_CART)
							RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);
						else
							RunValveToPossition_Bidirectional_AbortReady(V_CLEAN, VALVE_STEPS_PER_POSITION);

						if(BUBBLES_IN_TUBE)
							PumpVolume(FW, PumpVol_tube_bubble, Speed_Fast, 1);
						PumpVolume(FW, PumpVol_Clean, Speed_Fast, 1);
						userDelay(valve_delay, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						PumpVolume(FW, PumpVol_air_bubble + PumpVol_tube_bubble, Speed_Fast, 1);
						userDelay(valve_delay_after_air, 1);
						if(ISEs.Config == PH_CL_CART)
							RunValveToPossition_Bidirectional_AbortReady(V_RINSE, VALVE_STEPS_PER_POSITION);
						else
							RunValveToPossition_Bidirectional_AbortReady(V_CLEAN, VALVE_STEPS_PER_POSITION);
						PumpVolume(BW, PumpVol_tube_bubble, Speed_Fast, 1);
						userDelay(valve_delay, 1);
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						PumpVolume(FW, PumpVol_Clean_center, Speed_Fast, 1);

						SleepValve();

#ifdef CC_CURRENT_LIMITED
						CleanAmperometrics_CurrentLimited(0, 0, 0, OXIDE_REBUILD_TYPE);
#else
						CleanAmperometrics(0, 0, 0, OXIDE_REBUILD_TYPE);
#endif
					}

					if((gui32Error & 0) == 0)
						if(CONDITION_AMPS)
						{
							update_Status(STATUS_TEST, OPERATION_SAMPLE_B1);

							uint8_t mixing_index = 0;
//							uint8_t in_range = 0;	// Set once pH mixing gets into correct range

							if((gui32Error & 0) == 0)
							{
								// Prime a little B1 before test to clear out any contamination
								UARTprintf("Priming B1... \n");
								RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
								PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 1);
								userDelay(valve_delay_after_air, 1);
								RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);
								PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_Metering, 1);
								userDelay(valve_delay, 1);
							}

							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
							if(mixing_index == 0)
							{
								PumpVolume(FW, PumpVol_sample_rinse, Speed_Fast, 1);
							}
							//						FindPossitionZeroPump();
							userDelay(valve_delay, 1);

							mixing_index++;

							//
							// FCl, B1 Mixing
							//
							UARTprintf("Mixing %d uL of B1... \n", (int) PumpVol_Buffer);
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air

							if(CL_MIX_IN_AIR)
							{
								UARTprintf("Pumping large air plug so arrays and reference are uncovered during mixing\n");
								PumpVolume(FW, PumpVol_air_bubble + PumpVol_Clean + 16.8, Speed_Fast, 1);
							}
							else
								PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 1);
							FindPossitionZeroPump();
							userDelay(valve_delay_after_air, 1);

							// Pump buffer and solution
							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
							PumpStepperRunStepSpeed_AbortReady(FW, Steps_PreB1, Speed_Metering);
							userDelay(valve_delay_metering, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
							PumpVolume(FW, PumpVol_Buffer, Speed_Metering, 1);
							userDelay(valve_delay_metering, 1);

							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
							PumpVolume(FW, PumpVol_PostB1, Speed_Fast, 1);
							userDelay(valve_delay, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
							PumpVolume(FW, PumpVol_air_bubble /*+ PumpVol_tube_bubble*/, Speed_Fast, 1);
							//							userDelay(valve_delay_after_air, 1);

							//							RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
							//							PumpVolume(BW, PumpVol_tube_bubble, Speed_Metering, 1);
							//							userDelay(valve_delay_metering, 1);

							//						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to air
							//						PumpStepperRunStepSpeed_AbortReady(FW, Steps_follow_B1, Speed_placing);
							//						userDelay(valve_delay_metering, 1);

							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
							PumpStepperMix(BW, Steps_cycles, Speed_Fast, mix_cycles);
							//						for(i = 0; i < mix_cycles; i++)
							//						{
							//							PumpStepperRunStepSpeed_AbortReady(BW, Steps_cycles, Speed_mixing);
							//							PumpStepperRunStepSpeed_AbortReady(FW, Steps_cycles, Speed_mixing);
							//						}

							userDelay(diffusion_time, 1);

							//							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to air
							PumpVolume(FW, PumpVol_center, Speed_Fast, 1);

							SleepValve();

							if((gui32Error & 0) == 0)
							{
								if(1)
								{
									Cl_nA_FCl[k] = ReadClnA(HIGH_RANGE, Amp_Voltage_Set, CL_TRACE_TIME);

									UARTprintf("FCl raw: %d nA * 1000\n", (int) (Cl_nA_FCl[k] * 1000));

									T_Samp_B1 = MeasureTemperature(1);

									//									Cl_nA_FCl[k] /= 0.015 * T_Samp_B1 + 0.622;	// Normalize current to 25 C
									Cl_nA_FCl[k] /= 0.013 * T_Samp_B1 + 0.668;	// Normalize current to 25 C Updated 3/16/2020
									UARTprintf("FCl raw normalized to 25C: %d nA * 1000\n", (int) (Cl_nA_FCl[k] * 1000));

									//									float Conductivity_B1 = MeasureConductivity(Cond_EEP_Rinse, Cond_EEP_Cal_2, 0);
									UARTprintf("Temperature of mix: %d C * 1000\n", (int) (T_Samp_B1 * 1000));
									//									UARTprintf("Conductivity of B1 mix: %d uS/cm * 1000\n", (int) (Conductivity_B1 * 1000));

									//									if(Conductivity_B1 < 4000)
									//									{
									//										gui32Error |= FCL_MIX_OUT_OF_RANGE;	// Update error
									//										update_Error();
									//									}
								}
							}

							// Leave RE and CE floating
							IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);		// RE floating
							IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);		// CE floating

							RunValveToPossition_Bidirectional_AbortReady(V_B1, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
							PumpVolume(BW, PumpVol_tube_bubble, Speed_Metering, 1);
							userDelay(valve_delay_metering, 1);

						}


					//
					// TCL, B2
					//
					if((gui32Error & 0) == 0)
						if(CONDITION_AMPS)
						{
							update_Status(STATUS_TEST, OPERATION_SAMPLE_B2);

							uint8_t mixing_index = 0;
//							uint8_t in_range = 0;	// Set once pH mixing gets into correct range

							// Prime B2 at beginning only once, don't push back until after done mixing
							if((gui32Error & 0) == 0)
							{
								// Prime a little C2 before test to clear out any contamination
								UARTprintf("Priming C2... \n");
								RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
								PumpVolume(FW, PumpVol_air_bubble, Speed_Metering, 1);
								userDelay(valve_delay_after_air, 1);
								RunValveToPossition_Bidirectional_AbortReady(V_C2, VALVE_STEPS_PER_POSITION);
								PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_Metering, 1);
								userDelay(valve_delay, 1);

								//						// Prime a little B2 before test to clear out any contamination
								//						UARTprintf("Priming B2... \n");
								//						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
								//						PumpVolume(FW, PumpVol_air_bubble, Speed_Metering, 1);
								//						userDelay(valve_delay_after_air, 1);
								//						RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);
								//						PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_Metering, 1);
								//						userDelay(valve_delay, 1);
							}

							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
							PumpVolume(FW, 33.6, Speed_Fast, 1);	// Pump a normal sized plug of sample so if B2 or C2 plugs get stuck this sample will catch them
							userDelay(valve_delay, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
							PumpVolume(FW, PumpVol_air_bubble, Speed_Metering, 1);
							userDelay(valve_delay_after_air, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
							PumpVolume(FW, PumpVol_sample_rinse, Speed_Fast, 1);
							//						FindPossitionZeroPump();
							userDelay(valve_delay, 1);

							mixing_index++;

							// Pump and mix conditioner with sample
							UARTprintf("Mixing %d uL of C2... \n", (int) PumpVol_C2);

							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air

							if(CL_MIX_IN_AIR)
							{
								UARTprintf("Pumping large air plug so arrays and reference are uncovered during mixing\n");
								PumpVolume(FW, PumpVol_air_bubble + PumpVol_Clean, Speed_Fast, 1);
							}
							else
								PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 1);
							FindPossitionZeroPump();
							userDelay(valve_delay_after_air, 1);

							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);	// Move valve to sample
							PumpStepperRunStepSpeed_AbortReady(FW, Steps_PreB1, Speed_Metering);
							userDelay(valve_delay_metering, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_C2, VALVE_STEPS_PER_POSITION);		// Move valve to C2
							PumpVolume(FW, PumpVol_C2, Speed_Metering, 1);
							userDelay(valve_delay_metering, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);	// Move valve to sample
							PumpVolume(FW, PumpVol_PostB1, Speed_Metering, 1);
							userDelay(valve_delay_metering, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air

							PumpVolume(FW, PumpVol_air_bubble + PumpVol_tube_bubble, Speed_Metering, 1);
							userDelay(valve_delay_after_air, 1);

							// Push C2 back a little in valve to help avoid it contaminating B1 and causing us to lose FCl
							RunValveToPossition_Bidirectional_AbortReady(V_C2, VALVE_STEPS_PER_POSITION);		// Move valve to air
							PumpVolume(BW, PumpVol_tube_bubble, Speed_Metering, 1);
							userDelay(valve_delay, 1);

							//						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);	// Move valve to sample
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
							PumpVolume(FW, PumpVol_follow_C2, Speed_Fast, 1);
							//						userDelay(valve_delay_metering, 1);

							PumpStepperMix(BW, Steps_cycles, Speed_Fast, mix_cycles);

							userDelay(diffusion_time, 1);	// Delay to let diffusion happen

							// Prime a little B2 before test to clear out any contamination
							UARTprintf("Priming B2... \n");
							//						RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
							//						PumpVolume(FW, PumpVol_air_bubble, Speed_Metering, 1);
							//						userDelay(valve_delay_after_air, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);
							PumpVolume(FW, PumpVol_tube_prime_buffers, Speed_Metering, 1);
							userDelay(valve_delay, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);
							PumpVolume(FW, PumpVol_air_bubble, Speed_Metering, 1);
							userDelay(valve_delay_after_air, 1);

							// Pump mixed conditioner/solution back and mix with buffer
							UARTprintf("Mixing %d uL of B2... \n", (int) PumpVol_Buffer);

							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
							PumpVolume(BW, PumpVol_back + PumpVol_tube_prime_buffers + PumpVol_air_bubble, Speed_Metering, 1);
							userDelay(valve_delay, 1);
							int32_t Steps_to_align_pump = 1000 - (g_PumpStepsTravelled % 1000);
							if(Steps_to_align_pump == 1000)
								Steps_to_align_pump = 0;
							float Volume_to_align_pump = Steps_to_align_pump * PumpVolRev / Pump_Ratio / 1000;
							// Need to remove the dead spot volume, luckily with the PumpVolume code we will never stop the pump in the dead spot so it'll be an all or nothing calculation
							if(Steps_to_align_pump > 250)	// Check if we will be passing through dead spot, since we can't be inside the dead spot can merely check against center of dead spot
								Volume_to_align_pump -= (1 - Pump_Ratio) * PumpVolRev / Pump_Ratio;	// Remove the volume of the dead spot
							PumpStepperRunStepSpeed_AbortReady(FW, 1000 + Steps_to_align_pump, Speed_Metering);	// Buffer seemed to meter better if the pump was last pumping forward
							userDelay(valve_delay_metering, 1);

							RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);		// Move valve to buffer 1
							PumpVolume(FW, PumpVol_Buffer, Speed_Metering, 1);
							userDelay(valve_delay_metering, 1);

							RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);		// Move valve to sample
							PumpVolume(FW, PumpVol_forward - Volume_to_align_pump, Speed_Metering, 1);
							userDelay(valve_delay_metering, 1);

							PumpStepperMix(BW, Steps_cycles, Speed_Fast, mix_cycles);

							userDelay(diffusion_time, 1);	// Delay to let diffusion happen

							PumpVolume(FW, PumpVol_center_B2, Speed_Fast, 1);

							SleepValve();

							if((gui32Error & 0) == 0)
							{
								Cl_nA_TCl[k] = ReadClnA(HIGH_RANGE, Amp_Voltage_Set, CL_TRACE_TIME);

								UARTprintf("TCl raw: %d nA * 1000\n", (int) (Cl_nA_TCl[k] * 1000));

								T_Samp_B2 = MeasureTemperature(1);
								Cl_nA_TCl[k] /= 0.014 * (T_Samp_B2) + 0.654;
								UARTprintf("TCl raw normalized to 25C: %d nA * 1000\n", (int) (Cl_nA_TCl[k] * 1000));
								//								UARTprintf("TCl raw normalized to 25C: %d nA * 1000", (int) ((Cl_nA_TCl / (0.025832 * T_Samp_B2 + 0.354211)) * 1000));

								//									float Conductivity_B2 = MeasureConductivity(Cond_EEP_Rinse, Cond_EEP_Cal_2, 0);
								UARTprintf("Temperature of mix: %d C * 1000\n", (int) (T_Samp_B2 * 1000));
								//									UARTprintf("Conductivity of B2 mix: %d uS/cm * 1000\n", (int) (Conductivity_B2 * 1000));

								//									if(Conductivity_B2 < 9000)
								//									{
								//										gui32Error |= TCL_MIX_OUT_OF_RANGE;	// Update error
								//										update_Error();
								//									}

							}

							// RE and CE floating
							IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);		// Leave RE floating
							IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);		// Leave CE floating

							// After running TCl push B2 buffer back slightly into pouch
							RunValveToPossition_Bidirectional_AbortReady(V_AIR, VALVE_STEPS_PER_POSITION);		// Move valve to air
							PumpVolume(FW, PumpVol_air_bubble + PumpVol_tube_bubble, Speed_Metering, 1);
							userDelay(valve_delay_after_air, 1);
							RunValveToPossition_Bidirectional_AbortReady(V_B2, VALVE_STEPS_PER_POSITION);		// Move valve to air
							PumpVolume(BW, PumpVol_tube_bubble, Speed_Metering, 1);
							userDelay(valve_delay, 1);
						}


					if((CONDITION_AMPS) && (gui32Error & 0) == 0)	// Rinse with sample if chlorine was ran, skip if running alkalinity as it will be done there
					{
						UARTprintf("Rinsing with sample!\n");
						PrintTime();

						UARTprintf("Rinsing with one long sample plug!\n");
						RunValveToPossition_Bidirectional_AbortReady(V_SAMP, VALVE_STEPS_PER_POSITION);
						PumpVolume(FW, PumpVol_Sample_Prime, Speed_Fast, 1);
					}

					//				if((k == 2 || k == 5 || k == 8) && PURGE_SAMPLE)
					if(PURGE_SAMPLE)
					{
						UARTprintf("Purging sample tube by pushing air backwards into it\n");
						RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
						PumpVolume(FW, PumpVol_sample_rinse + PumpVol_Sample_Prime + 33.6, Speed_Fast, 1);
						userDelay(valve_delay_after_air, 1);
						RunValveToPossition_Bidirectional(V_SAMP, VALVE_STEPS_PER_POSITION);
						PumpVolume(BW, PumpVol_Sample_Prime, Speed_Fast, 1);
						userDelay(valve_delay, 1);
					}
					else
					{
						UARTprintf("Not purging sample tube\n");
						RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
						PumpVolume(FW, PumpVol_sample_rinse, Speed_Fast, 0);
						userDelay(valve_delay_after_air, 0);
						if(BUBBLES_IN_TUBE)
						{
							RunValveToPossition_Bidirectional(V_SAMP, VALVE_STEPS_PER_POSITION);
							PumpVolume(BW, PumpVol_tube_bubble, Speed_Fast, 0);
							userDelay(valve_delay, 0);
						}
					}

					SleepValve();

					TestValveDrift();
				}	// For all 9 samples


			if((CONDITION_AMPS || CALIBRATE_CONDUCTIVITY) /*&& CALIBRATE_ISES*/ == 0)
			{
				struct SolutionVals* Sols = FillSolutionStruct();
				uint8_t Storage_Port;
				if(ISEs.Config == PH_CL_CART && Sols->Cond_EEP_Cal_2 > 900)	// pH only cartridge with pH 9 clean in place of Cal 2
				{
					Storage_Port = V_CAL_2;
					UARTprintf("Storing from Cal 2 port\n");
				}
				else if(Sols->pH_EEP_Clean < 8.5)
				{
					Storage_Port = V_RINSE;
					UARTprintf("Storing from Rinse port\n");
				}
				else
				{
					Storage_Port = V_CLEAN;
					UARTprintf("Storing from Clean port\n");
				}

				//
				// Flow Chart teal section, post-rinse
				//
				PrintTime();
				if((gui32Error & 0) == 0)
				{
					update_Status(STATUS_TEST, OPERATION_TEST_POSTCHECK);
					UARTprintf("Pumping Postrinse... \n");

					RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
					FindPossitionZeroPump();
					for (i = 0; i < Number_of_bubbles_Postrinse; i++) // Loop over air/solution cycle 3 times for single solution
					{
						RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 0);
						if(i == (Number_of_bubbles_Postrinse - 1))
							PumpVolume(FW, PumpVol_Large_air_bubble, Speed_Fast, 0);
						userDelay(valve_delay_after_air,0);

						RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);

						if(i == 0 && BUBBLES_IN_TUBE)
							PumpVolume(FW, PumpVol_tube_bubble, Speed_Fast, 0);
						PumpVolume(FW, PumpVol_Solution, Speed_Fast, 0);
						if(i != (Number_of_bubbles_Postrinse - 1))
							userDelay(valve_delay, 0);

						if((gui32Error & 0) != 0)	// If abort command is received stop bubble cycle and just fill with rinse
							break;
					}
					if(STORE_HUMID == 1)
					{
						UARTprintf("Storing with air over all sensors!\n");
						RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						PumpVolume(FW, PumpVol_Solution + PumpVol_Rinse, Speed_Fast, 0);
						userDelay(valve_delay, 0);
						RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
						PumpVolume(FW, PumpVol_plug, Speed_Fast, 0);
					}
					else if(STORE_FRIT_DRY == 1)
					{
						UARTprintf("Storing with air over reference!\n");
						RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
						PumpVolume(FW, 134.4, Speed_Fast, 0);
					}
//					else if(STORE_AMPS_DRY == 1)
//					{
//						UARTprintf("Storing air bubble over amperometrics only!\n");
//						RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
//						PumpVolume(FW, PumpVol_air_bubble, Speed_Fast, 0);
//						userDelay(valve_delay, 1);
//						RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
//						PumpVolume(FW, 117.6, Speed_Fast, 0);
//						userDelay(valve_delay, 0);
//
//						// Set RE and CE floating and close RE/CE loop
//						IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWA, 1);
//						IO_Ext_Set(IO_EXT1_ADDR, 3, REF_EL_SWB, 0);
//
//						// 10.7 uApp R = 309k + 499k = 808k
//						IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWA, 0);
//						IO_Ext_Set(IO_EXT1_ADDR, 3, COND_GAIN_SWB, 0);
//
//						WaveGenSet(1);
//						userDelay(100, 1);
//						uint8_t signal_doubled = 0;
//						uint8_t checkstep = 0;
//
//						float CondReading = ConductivityMovingAvg(COND_FREQ);
//						float OldReading = CondReading;
//
//						UARTprintf("Reading\t%d\n", (int) CondReading);
//						while(signal_doubled == 0 && checkstep <= 60)
//						{
//							PumpVolume(FW, 2.75, Speed_Fast, 0);
//							checkstep++;
//
//							CondReading = ConductivityMovingAvg(COND_FREQ);
//							UARTprintf("Reading\t%d\n", (int) CondReading);
//
//							if(CondReading/OldReading > 2)
//							{
//								UARTprintf("Conductivity signal doubled! Found storage location!\n");
//								signal_doubled = 1;
//							}
//							OldReading = CondReading;
//
//
//						}
//						if(checkstep > 37)
//						{
//							UARTprintf("Didn't find storage location!\n");
//							PumpVolume(FW, 33.6, Speed_Fast, 0);
//						}
//
//						WaveGenSet(0);
//					}
					else
					{
						UARTprintf("Storing covering everyting!\n");
						PumpVolume(FW, PumpVol_Rinse, Speed_Fast, 0);
						if(FLOOD_TO_STORE == 0)
						{
							userDelay(valve_delay, 0);
							RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);		// Always start with air purge
						}
						PumpVolume(FW, PumpVol_plug, Speed_Fast, 0);
					}

					// Push air back into rinse port before moving to next solution
					if(BUBBLES_IN_TUBE)
					{
						RunValveToPossition_Bidirectional(V_AIR, VALVE_STEPS_PER_POSITION);
						PumpVolume(FW, PumpVol_tube_bubble * 4, Speed_Fast, 0);
						userDelay(valve_delay_after_air, 0);
						RunValveToPossition_Bidirectional(Storage_Port, VALVE_STEPS_PER_POSITION);
						PumpVolume(BW, PumpVol_tube_bubble, Speed_Fast, 0);
						userDelay(valve_delay, 0);
					}

					SleepValve();
				}

				gui32Error &= ~ROAM_RESET;	// Turn off ROAM_RESET flag before saving error code
				UARTprintf("Error Code: 0x%x\n\n", gui32Error);
				PrintErrors(gui32Error, 1, STATE_MEASUREMENT);

				FindPossitionOneValve();
			}

			BuzzerSound(400);
			SetLED(BLUE_BUTTON_BLINK, 1);

			UARTprintf("\n\n");

			// Pause at end to wait for user to empty waste chamber
			update_Status(STATUS_TEST, OPERATION_TEST_EMPTY_WASTE);
//			UARTprintf("Empty waste chamber! Press button to continue. \n");

			SetLED(GREEN_BUTTON_BLINK | BLUE_BUTTON_BLINK, 0);
			if(FactoryCalCheck == 1)
				SetLED(GREEN_BUTTON | GREEN_BUTTON_V, 1);
			else
				SetLED(RED_BUTTON | RED_BUTTON_V, 1);

			if(ThermCheck == 0)
				UARTprintf("WARNING! Thermistor failed calibration!\n");
			else if(CALIBRATE_THERM)
				UARTprintf("Thermistor slope: %d\n", (int) (Therm_corr * 1000));

			if(CondCheck == 0)
				UARTprintf("WARNING! Conductivity failed calibration!\n");
			else if(CALIBRATE_CONDUCTIVITY)
				UARTprintf("Conductivity Slope: %d\n", (int) (Cond_Slope * 1000));


			counter = 0;
			while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == GPIO_PIN_3 && counter < TIMEOUT)
			{
				SysCtlDelay(SysCtlClockGet()/3000);
				counter++;

				// Break out of loop if another start test command is received
				if(g_ui32DataRx0[0] == CONTINUE_TEST && g_ulSSI0RXTO > 0)
				{
					g_ulSSI0RXTO = 0;
					break;
				}

				// Check if state changed, this happens when abort command is received
				if(g_state != STATE_FACTORY_CAL)
					break;
			}

//			update_Status(STATUS_TEST, OPERATION_TEST_COMPLETE);
//			UARTprintf("Test completed! Error Code: 0x%x\n", gui32Error);

			if((gui32Error & (MAX_TESTS_REACHED|CARTRIDGE_EXPIRED|BATTERY_TOO_LOW|I2C_FAILED/*|SPI_FAILED*/|MEMORY_FAILED|USER_CANCELLED)) == 0 || DEMO_UNIT == 1)
			{
				update_Status(STATUS_TEST, OPERATION_TEST_COMPLETE);
//				UARTprintf("Test completed! Error Code: 0x%x\n", gui32Error);
			}
			else
			{
				update_Status(STATUS_TEST, OPERATION_TEST_FAILED);
//				UARTprintf("Test failed! Error Code: 0x%x\n", gui32Error);
			}
//			update_Status(STATUS_TEST, OPERATION_TEST_COMPLETE);
			while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == 0x00);

//			// Wait 3 seconds to let app read status and operation before switching back to idle
//			SysCtlDelay(SysCtlClockGet());

			counter = 0;
			while(counter < TIMEOUT && GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == GPIO_PIN_3)
			{
				SysCtlDelay(SysCtlClockGet()/3000);
				counter++;

				// Break out of loop if continue test command is received
				if(g_ui32DataRx0[0] == CONTINUE_TEST && g_ulSSI0RXTO > 0)
				{
					g_ulSSI0RXTO = 0;
					break;
				}

				// Check if state changed, this happens when abort command is received
				if(g_state != STATE_FACTORY_CAL)
					break;
			}
			while(GPIOPinRead(GPIO_PORTF_BASE, GPIO_PIN_3) == 0);

			g_state = STATE_IDLE;
			break;
		}
#endif
		default:
		{
			g_state = STATE_IDLE;
			g_next_state = STATE_IDLE;
			break;
		}

		} // switch

	} // while

} // main
