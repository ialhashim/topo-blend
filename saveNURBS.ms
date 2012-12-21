--====================================================
--
--   Convert to simple curve
--
--====================================================
try destroyDialog simpleCurver catch()

rollout saveNURBS "Save NURBS to XML"
(
	group "Options"
	(
		--spinner np "Num Points:" range:[0,1000,30] type:#integer fieldwidth:30 align:#left
		--spinner scaled "Scale:" range:[0.0,1000.0,1.0] type:#float fieldwidth:30 align:#left
	)
	
	group "Export"
	(
		button saveButton "Save.."
	)
	
	group "Import"
	(
		button loadButton "Load.."
	)

	fn saveShape =
	(
		fileName = getSaveFileName caption:"Save Curve" types:"XML NURBS file (*.xml)|*.xml|All Files (*.*)|*.*|"
		fs = createFile fileName
		
		-- Write XML header
		format "<?xml version=\"1.0\" encoding=\"UTF-8\" ?>\n<document>\n" to: fs
		
		-- Collect all NURB curves
		deselect $*
		
		--points = #() 
		-- add points
		for obj in $* do
		(
			-- Figure out class types and stuff
			objectClass = (classof obj)
			if(not (objectClass == NURBSCurveshape or objectClass == NURBSSurf)) then continue;
			t = obj.transform
				
			nSet = getNURBSSet obj 
			curve = nSet[1]
			
			-- Get dimentions
			knotCount = curve.numCVs
			
			format "<node>\n" to: fs
				
			-- Write name
			format "\t<id>%</id>\n" obj.name to: fs
			
			-- Assign type
			nurbType = ""
			if (objectClass == NURBSCurveshape) then nurbType = "CURVE"
			if (objectClass == NURBSSurf) then nurbType = "SHEET"
			format "\t<type>%</type>\n" nurbType to: fs
			
			if(objectClass == NURBSCurveshape) then 
			(
				-- Write control count
				format "\t<controls>\n" to: fs
				format "\t<c>%</c>\n" knotCount to: fs
				format "\t</controls>\n" to: fs
				
				weightsArray = #()
				
				-- Print control points positions
				for i = 1 to knotCount do
				(
					cvpoint = getCV curve i
					p = cvpoint.pos * t
					format "\t<point>% % %</point>\n" p.x p.y p.z to: fs
					
					w = cvpoint.weight
					append weightsArray w
				)
				
				-- Print control point weight
				format "\t<weights>" to: fs
				for w in weightsArray do
				(
					format "% " w to: fs
				)
				format "</weights>\n" to: fs
			)
			
			if(objectClass == NURBSSurf) then 
			(
				-- Write control count
				format "\t<controls>\n" to: fs
				format "\t<c>%</c>\n" (knotCount[1] as integer) to: fs
				format "\t<c>%</c>\n" (knotCount[2] as integer) to: fs
				format "\t</controls>\n" to: fs
				
				weightsArray = #()
				
				-- Print control points positions
				for u = 1 to knotCount[1] do
				(
					for v = 1 to knotCount[2] do
					(
						cvpoint = getCV curve u v
						p = cvpoint.pos * t
						format "\t<point>% % %</point>\n" p.x p.y p.z to: fs
						
						w = cvpoint.weight
						append weightsArray w
					)
				)
				
				-- Print control point weight
				format "\t<weights>" to: fs
				for w in weightsArray do
				(
					format "% " w to: fs
				)
				format "</weights>\n" to: fs
			)
			
			format "</node>\n" to: fs
		)
		
		-- Write XML footer
		format "</document>" to: fs
		
		close fs
	)
	
	fn loadShape = 
	(
		fileName = getOpenFileName caption:"Open NURBS" types:"NURBS file (*.xml)|*.xml|All Files (*.*)|*.*|"
		fs = openFile fileName
	)
	
	on saveButton pressed do 
	(
		saveShape()
	)
	
	on loadButton pressed do 
	(
		loadShape()
	)
)

createDialog saveNURBS


