#include "Synthesizer.h"

Synthesizer::Synthesizer()
{
}

void Synthesizer::resampleCurve(Curve *structure_curve, int uResolution, int thetaResolution, int phiResolution)
{
    if(!structure_curve->property.contains("mesh")) return;

    // Data structure:
    SurfaceMeshModel * mesh = structure_curve->property["mesh"].value<SurfaceMeshModel*>();
    SurfaceMeshModel * resampledMesh = new SurfaceMeshModel(structure_curve->id+".off", structure_curve->id);

    structure_curve->property["resampled"].setValue( resampledMesh );

    Octree * octree = NULL;
    if(structure_curve->property.contains("octree"))
        octree = structure_curve->property["octree"].value<Octree*>();
    if(!octree)
    {
        std::vector<Surface_mesh::Face> allTris;
        foreach(Face f, mesh->faces()) allTris.push_back(f);
        int numTrisPerNode = 50;
        octree = new Octree(numTrisPerNode, mesh);
        octree->initBuild(allTris, numTrisPerNode);
        structure_curve->property["octree"].setValue(octree);
    }

    NURBSCurve & curve = structure_curve->curve;

    Array2D_Vector3 faces;
    std::vector<Vertex> verticesIdx;

    // Go over faces
    Vector3VertexProperty points = mesh->vertex_property<Vector3>(VPOINT);
    foreach(Face f, mesh->faces())
    {
        // Collect its points
        Surface_mesh::Vertex_around_face_circulator vit = mesh->vertices(f),vend=vit;
        std::vector<Vector3> face_vertices;
        do{ face_vertices.push_back(points[vit]); } while(++vit != vend);
        faces.push_back(face_vertices);
    }

    Vector3 initialDirection;
    Vector3 initialTangent;
    double thetaRange = 2*M_PI;
    double phiRange = M_PI/2;
    int sampling_thetaResolution = (int)thetaResolution*thetaRange /(2*M_PI);
    int sampling_phiResolution = (int)phiResolution*phiRange /(M_PI);   // full range of phi here is M_PI

    // set the initial theta sampling direction as the curve binormal
    initialDirection = curve.GetBinormal(0);
    initialTangent = curve.GetTangent(0);

    // Resample along curve
    Array2D_Vector3 crossSections = cylinderResampling(faces, curve, initialDirection,
                                                       uResolution, sampling_thetaResolution, thetaRange, octree, false);

    int idxBase = verticesIdx.size();
    addCylinderFaces(crossSections, resampledMesh, verticesIdx, idxBase);

    int tNum = crossSections.size();
    int thetaNum = crossSections[0].size();

    for(int curr_t = 0; curr_t < tNum-1; curr_t++)
    {
        std::vector<Vertex> face_vertex_idx;
        face_vertex_idx.push_back(verticesIdx[curr_t*thetaNum+thetaNum-1+idxBase]);
        face_vertex_idx.push_back(verticesIdx[curr_t*thetaNum+idxBase]);
        face_vertex_idx.push_back(verticesIdx[(curr_t+1)*thetaNum+idxBase]);
        face_vertex_idx.push_back(verticesIdx[(curr_t+1)*thetaNum+thetaNum-1+idxBase]);

        resampledMesh->add_face(face_vertex_idx);
    }

    // Resample curve ends
    Vector3 pos, Tangent, thetaStart;

    // t = 0
    pos = curve.GetPosition(0);
    Tangent = curve.GetTangent(0);
    thetaStart = cross(initialDirection, Tangent);

    Array2D_Vector3 endResamplings1 = sphereResampling(faces, pos, thetaStart, -Tangent,
        sampling_phiResolution, sampling_thetaResolution, phiRange, thetaRange, octree);
    addEndFaces(endResamplings1, resampledMesh, verticesIdx, verticesIdx.size());

    // t = 1
    pos = curve.GetPosition(1);
    Tangent = curve.GetTangent(1);
    thetaStart = -cross(initialDirection, Tangent);

    Array2D_Vector3 endResamplings2 = sphereResampling(faces, pos, thetaStart, Tangent,
        sampling_phiResolution, sampling_thetaResolution, phiRange, thetaRange, octree);
    addEndFaces(endResamplings2, resampledMesh, verticesIdx, verticesIdx.size());

    // Stitch cylinder:

    // connect the end with the trunk
    int numTrunk = (uResolution-1)*thetaResolution;
    int numEnd = thetaResolution*(sampling_phiResolution + 1);
    int currentTotal = verticesIdx.size();

    std::vector<Vertex> face_vertex_idx;
    // t = 0, the rotation of sphere sampling is inverse to the trunk
    // curve: startDirection=biNormal
    face_vertex_idx.push_back(verticesIdx[currentTotal-2*numEnd+sampling_phiResolution*thetaResolution+1]);
    face_vertex_idx.push_back(verticesIdx[currentTotal-2*numEnd+sampling_phiResolution*thetaResolution]);
    face_vertex_idx.push_back(verticesIdx[currentTotal-2*numEnd-numTrunk]);
    face_vertex_idx.push_back(verticesIdx[currentTotal-2*numEnd-numTrunk+ thetaResolution-1]);

    resampledMesh->add_face(face_vertex_idx);

    for (int curr_theta=1; curr_theta< thetaResolution-1; curr_theta++)
    {
        std::vector<Vertex> face_vertex_idx;

        face_vertex_idx.push_back(verticesIdx[currentTotal-2*numEnd+sampling_phiResolution*thetaResolution+curr_theta+1]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-2*numEnd+sampling_phiResolution*thetaResolution+curr_theta]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-2*numEnd-numTrunk+thetaResolution-curr_theta]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-2*numEnd-numTrunk+thetaResolution-curr_theta-1]);

        resampledMesh->add_face(face_vertex_idx);
    }

    face_vertex_idx.clear();
    face_vertex_idx.push_back(verticesIdx[currentTotal-2*numEnd+sampling_phiResolution*thetaResolution]);
    face_vertex_idx.push_back(verticesIdx[currentTotal-2*numEnd+sampling_phiResolution*thetaResolution+thetaResolution-1]);
    face_vertex_idx.push_back(verticesIdx[currentTotal-2*numEnd-numTrunk+1]);
    face_vertex_idx.push_back(verticesIdx[currentTotal-2*numEnd-numTrunk]);

    resampledMesh->add_face(face_vertex_idx);

    //t = 1
    for (int curr_theta=0; curr_theta< thetaResolution-1; curr_theta++)
    {
        std::vector<Vertex> face_vertex_idx;
        face_vertex_idx.push_back(verticesIdx[currentTotal-2*numEnd-thetaResolution+curr_theta]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-2*numEnd-thetaResolution+curr_theta+1]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-numEnd+sampling_phiResolution*thetaResolution+curr_theta+1]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-numEnd+sampling_phiResolution*thetaResolution+curr_theta]);

        resampledMesh->add_face(face_vertex_idx);
    }

    face_vertex_idx.clear();
    face_vertex_idx.push_back(verticesIdx[currentTotal-2*numEnd-thetaResolution+thetaResolution-1]);
    face_vertex_idx.push_back(verticesIdx[currentTotal-2*numEnd-thetaResolution]);
    face_vertex_idx.push_back(verticesIdx[currentTotal-numEnd+sampling_phiResolution*thetaResolution]);
    face_vertex_idx.push_back(verticesIdx[currentTotal-numEnd+sampling_phiResolution*thetaResolution+thetaResolution-1]);

    resampledMesh->add_face(face_vertex_idx);
}

void Synthesizer::resampleSheet(Sheet *structure_sheet, int uResolution, int vResolution, int thetaResolution, int phiResolution)
{
    if(!structure_sheet->property.contains("mesh")) return;

    // Data structure:
    SurfaceMeshModel * mesh = structure_sheet->property["mesh"].value<SurfaceMeshModel*>();
    SurfaceMeshModel * resampledMesh = new SurfaceMeshModel(structure_sheet->id+".off", structure_sheet->id);

    structure_sheet->property["resampled"].setValue( resampledMesh );

    Octree * octree = NULL;
    if(structure_sheet->property.contains("octree"))
        octree = structure_sheet->property["octree"].value<Octree*>();
    if(!octree)
    {
        std::vector<Surface_mesh::Face> allTris;
        foreach(Face f, mesh->faces()) allTris.push_back(f);
        int numTrisPerNode = 50;
        octree = new Octree(numTrisPerNode, mesh);
        octree->initBuild(allTris, numTrisPerNode);
        structure_sheet->property["octree"].setValue(octree);
    }

    NURBSRectangle & sheet = structure_sheet->surface;

    Array2D_Vector3 faces;
    std::vector<Vertex> verticesIdx;

    // Go over faces
    Vector3VertexProperty points = mesh->vertex_property<Vector3>(VPOINT);
    foreach(Face f, mesh->faces())
    {
        // Collect its points
        Surface_mesh::Vertex_around_face_circulator vit = mesh->vertices(f),vend=vit;
        std::vector<Vector3> face_vertices;
        do{ face_vertices.push_back(points[vit]); } while(++vit != vend);
        faces.push_back(face_vertices);
    }

    QElapsedTimer resamplingTimer; resamplingTimer.start();

    // align initial sampling direction
    Vector3 position, sheetNormal, tangentU, tangentV;
    Vector3 initialDirection;
    Vector3 origin;
    sheet.GetFrame(0, 0, origin, tangentU, tangentV, sheetNormal);
    initialDirection = sheetNormal;

    std::vector<Array2D_Vector3> resampledPlane = planeResamping(faces, sheet, initialDirection, uResolution, vResolution, octree);
    addPlaneFaces(resampledPlane, resampledMesh, verticesIdx, verticesIdx.size());

    double t1=resamplingTimer.elapsed();
    qDebug() << QString("Plane resampling =%1 ms").arg(t1);

    QElapsedTimer cylinderTimer; cylinderTimer.start();

    // Boundary curve initialization
    //Vector3 initialDirection, initialTargetDirection;
    double thetaRange = M_PI;
    double phiRange = M_PI/2;
    int sampling_thetaResolution = (int)thetaResolution*thetaRange/(2*M_PI);
    int sampling_phiResolution = (int)phiResolution*phiRange/(2*M_PI);

    std::vector<Vector3> ctrlPoints1=sheet.GetControlPointsU(0);
    std::vector<Real> ctrlWeights1(ctrlPoints1.size(), 1.0);
    NURBSCurve curve1=NURBSCurve(ctrlPoints1, ctrlWeights1, 3, false, true);

    std::vector<Vector3> ctrlPoints2=sheet.GetControlPointsV(sheet.mNumVCtrlPoints-1);
    std::vector<Real> ctrlWeights2(ctrlPoints2.size(), 1.0);
    NURBSCurve curve2=NURBSCurve(ctrlPoints2, ctrlWeights2, 3, false, true);

    std::vector<Vector3> ctrlPoints3=sheet.GetControlPointsU(sheet.mNumUCtrlPoints-1);
    std::vector<Real> ctrlWeights3(ctrlPoints3.size(), 1.0);
    NURBSCurve curve3=NURBSCurve(ctrlPoints3, ctrlWeights3, 3, false, true);

    std::vector<Vector3> ctrlPoints4=sheet.GetControlPointsV();
    std::vector<Real> ctrlWeights4(ctrlPoints4.size(), 1.0);
    NURBSCurve curve4=NURBSCurve(ctrlPoints4, ctrlWeights4, 3, false, true);

    double ct1=cylinderTimer.elapsed();
    qDebug() << "	"<<QString("boundary cylinder initialization =%1ms").arg(ct1);

    // Start cylinder sampling at the boundaries
    // sampling using the sheet boundaries, counter-clockwise

    // curve1: transverse u while v=0
    Array2D_Vector3 crossSections1=cylinderResampling(faces,curve1,initialDirection,
        uResolution,sampling_thetaResolution,thetaRange, octree, true);

    // curve2: transverse v while u=uResolution
    Array2D_Vector3 crossSections2=cylinderResampling(faces,curve2,initialDirection,
        vResolution,sampling_thetaResolution,thetaRange, octree, true);

    // curve3: transverse u while v=vResolution
    initialDirection = -initialDirection;
    Array2D_Vector3 crossSections3=cylinderResampling(faces,curve3,initialDirection,
        uResolution,sampling_thetaResolution,thetaRange, octree,true);

    // curve4: transverse v while u=0
    Array2D_Vector3 crossSections4=cylinderResampling(faces,curve4,initialDirection,
        vResolution,sampling_thetaResolution,thetaRange,octree,true);

    double ct2 = cylinderTimer.elapsed();

    // add faces to the mesh
    addCylinderFaces(crossSections1, resampledMesh, verticesIdx, verticesIdx.size());
    addCylinderFaces(crossSections2, resampledMesh, verticesIdx, verticesIdx.size());
    addCylinderFaces(crossSections3, resampledMesh, verticesIdx, verticesIdx.size());
    addCylinderFaces(crossSections4, resampledMesh, verticesIdx, verticesIdx.size());

    double ct3=cylinderTimer.elapsed();
    qDebug() << "	"<<QString("cylinder resampling =%1 ms add face = %2ms").arg(ct2-ct1).arg(ct3-ct2);

    double t2=resamplingTimer.elapsed();
    qDebug() << QString("Boundary cylinder resampling =%1 ms").arg(t2-t1);

    // corner1: u=0, v=0
    sheet.GetFrame(0,0,position, tangentU, tangentV, sheetNormal);
    initialDirection=sheetNormal;

    Array2D_Vector3 cornerResamplings1 = sphereResampling(faces,position,-tangentV,initialDirection,
        sampling_thetaResolution,sampling_phiResolution, thetaRange, phiRange, octree);

    addCornerFaces(cornerResamplings1, resampledMesh, verticesIdx, verticesIdx.size());

    // corner2: u=1, v=0
    sheet.GetFrame(1,0,position, tangentU, tangentV, sheetNormal);
    if (dot(initialDirection, sheetNormal)<0)
        sheetNormal=-sheetNormal;

    Array2D_Vector3 cornerResamplings2=sphereResampling(faces,position,tangentU,sheetNormal,
        sampling_thetaResolution,sampling_phiResolution, thetaRange, phiRange, octree);

    addCornerFaces(cornerResamplings2, resampledMesh, verticesIdx, verticesIdx.size());

    // corner3: u=1, v=1
    sheet.GetFrame(1,1,position, tangentU, tangentV, sheetNormal);
    if (dot(initialDirection, sheetNormal)<0)
        sheetNormal=-sheetNormal;

    Array2D_Vector3 cornerResamplings3=sphereResampling(faces,position,tangentV,sheetNormal,
        sampling_thetaResolution,sampling_phiResolution, thetaRange, phiRange, octree);

    addCornerFaces(cornerResamplings3, resampledMesh, verticesIdx, verticesIdx.size());

    // corner4: u=0, v=1
    sheet.GetFrame(0,1,position, tangentU, tangentV, sheetNormal);
    if (dot(initialDirection, sheetNormal)<0)
    {
        sheetNormal=-sheetNormal;
    }
    Array2D_Vector3 cornerResamplings4=sphereResampling(faces,position,-tangentU,sheetNormal,
        sampling_thetaResolution,sampling_phiResolution, thetaRange, phiRange, octree);

    addCornerFaces(cornerResamplings4, resampledMesh, verticesIdx, verticesIdx.size());

    // Stitch plane:
    int numCorner=(sampling_thetaResolution+1)*(sampling_phiResolution+1);
    int numTrunckU=(uResolution-1)*(sampling_thetaResolution+1);
    int numTrunckV=(vResolution-1)*(sampling_thetaResolution+1);
    int numOnePlane=(uResolution-1)*(vResolution-1);
    int currentTotal=verticesIdx.size();
    // connect plane to boundary cylinder
    // curve1: v=0
    for (int curr_u=0; curr_u< uResolution-2; curr_u++)
    {
        std::vector<Vertex> face_vertex_idx;
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-2*numOnePlane+curr_u]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU+curr_u*(sampling_thetaResolution+1)]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU+(curr_u+1)*(sampling_thetaResolution+1)]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-2*numOnePlane+curr_u+1]);

        resampledMesh->add_face(face_vertex_idx);
    }

    for (int curr_u=0; curr_u< uResolution-2; curr_u++)
    {
        std::vector<Vertex> face_vertex_idx;
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-numOnePlane+curr_u]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-numOnePlane+curr_u+1]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU+(curr_u+1)*(sampling_thetaResolution+1)+sampling_thetaResolution]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU+curr_u*(sampling_thetaResolution+1)+sampling_thetaResolution]);

        resampledMesh->add_face(face_vertex_idx);
    }

    // curve2: u=uResolution
    for (int curr_v=0; curr_v< vResolution-2; curr_v++)
    {
        std::vector<Vertex> face_vertex_idx;
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-2*numOnePlane+curr_v*(uResolution-1)+uResolution-2]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-numTrunckU+curr_v*(sampling_thetaResolution+1)]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-numTrunckU+(curr_v+1)*(sampling_thetaResolution+1)]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-2*numOnePlane+(curr_v+1)*(uResolution-1)+uResolution-2]);

        resampledMesh->add_face(face_vertex_idx);
    }

    for (int curr_v=0; curr_v< vResolution-2; curr_v++)
    {
        std::vector<Vertex> face_vertex_idx;
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-numOnePlane+curr_v*(uResolution-1)+uResolution-2]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-numOnePlane+(curr_v+1)*(uResolution-1)+uResolution-2]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-numTrunckU+(curr_v+1)*(sampling_thetaResolution+1)+sampling_thetaResolution]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-numTrunckU+curr_v*(sampling_thetaResolution+1)+sampling_thetaResolution]);

        resampledMesh->add_face(face_vertex_idx);
    }

    // curve3: v=vResolution
    for (int curr_u=0; curr_u< uResolution-2; curr_u++)
    {
        std::vector<Vertex> face_vertex_idx;
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-2*numOnePlane+(uResolution-1)*(vResolution-2)+curr_u]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-2*numOnePlane+(uResolution-1)*(vResolution-2)+curr_u+1]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckV-numTrunckU+(curr_u+1)*(sampling_thetaResolution+1)+sampling_thetaResolution]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckV-numTrunckU+(curr_u)*(sampling_thetaResolution+1)+sampling_thetaResolution]);

        resampledMesh->add_face(face_vertex_idx);
    }

    for (int curr_u=0; curr_u< uResolution-2; curr_u++)
    {
        std::vector<Vertex> face_vertex_idx;
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-numOnePlane+(uResolution-1)*(vResolution-2)+curr_u]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckV-numTrunckU+curr_u*(sampling_thetaResolution+1)]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckV-numTrunckU+(curr_u+1)*(sampling_thetaResolution+1)]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-numOnePlane+(uResolution-1)*(vResolution-2)+curr_u+1]);

        resampledMesh->add_face(face_vertex_idx);
    }

    // curve 4 u=0;
    for (int curr_v=0; curr_v< vResolution-2; curr_v++)
    {
        std::vector<Vertex> face_vertex_idx;
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-2*numOnePlane+curr_v*(uResolution-1)]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-2*numOnePlane+(curr_v+1)*(uResolution-1)]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckV+(curr_v+1)*(sampling_thetaResolution+1)+sampling_thetaResolution]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckV+curr_v*(sampling_thetaResolution+1)+sampling_thetaResolution]);

        resampledMesh->add_face(face_vertex_idx);
    }

    for (int curr_v=0; curr_v< vResolution-2; curr_v++)
    {
        std::vector<Vertex> face_vertex_idx;
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-numOnePlane+curr_v*(uResolution-1)]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckV+curr_v*(sampling_thetaResolution+1)]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckV+(curr_v+1)*(sampling_thetaResolution+1)]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU-numOnePlane+(curr_v+1)*(uResolution-1)]);

        resampledMesh->add_face(face_vertex_idx);
    }


    // connect corner to boundary cylinder
    // corner1: u=0, v=0
    // corner1:samplingStartDirection=sheetNormal; curve4: samplingStartDirection=-sheetNormal;
    for (int curr_theta=0; curr_theta< sampling_thetaResolution; curr_theta++)
    {
        std::vector<Vertex> face_vertex_idx;
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner+curr_theta*(sampling_phiResolution+1)]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckV+sampling_thetaResolution-curr_theta]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckV+sampling_thetaResolution-curr_theta-1]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner+(curr_theta+1)*(sampling_phiResolution+1)]);

        resampledMesh->add_face(face_vertex_idx);
    }

    // corner1:samplingStartDirection=sheetNormal; curve1: samplingStartDirection=sheetNormal;
    for (int curr_theta=0; curr_theta< sampling_thetaResolution; curr_theta++)
    {
        std::vector<Vertex> face_vertex_idx;
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner+curr_theta*(sampling_phiResolution+1)+sampling_phiResolution]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner+(curr_theta+1)*(sampling_phiResolution+1)+sampling_phiResolution]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU+curr_theta+1]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU+curr_theta]);

        resampledMesh->add_face(face_vertex_idx);
    }

    // corner2: u=1, v=0
    // corner2:samplingStartDirection=sheetNormal; curve1: samplingStartDirection=sheetNormal;
    for (int curr_theta=0; curr_theta< sampling_thetaResolution; curr_theta++)
    {
        std::vector<Vertex> face_vertex_idx;
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU+(uResolution-2)*(sampling_thetaResolution+1)+curr_theta]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-2*numTrunckU+(uResolution-2)*(sampling_thetaResolution+1)+curr_theta+1]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-3*numCorner+(curr_theta+1)*(sampling_phiResolution+1)]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-3*numCorner+curr_theta*(sampling_phiResolution+1)]);

        resampledMesh->add_face(face_vertex_idx);
    }

    // corner2:samplingStartDirection=sheetNormal; curve2: samplingStartDirection=sheetNormal;
    for (int curr_theta=0; curr_theta< sampling_thetaResolution; curr_theta++)
    {
        std::vector<Vertex> face_vertex_idx;
        face_vertex_idx.push_back(verticesIdx[currentTotal-3*numCorner+curr_theta*(sampling_phiResolution+1)+sampling_phiResolution]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-3*numCorner+(curr_theta+1)*(sampling_phiResolution+1)+sampling_phiResolution]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-numTrunckU+curr_theta+1]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-numTrunckU+curr_theta]);

        resampledMesh->add_face(face_vertex_idx);
    }

    // corner3: u=1, v=1
    // corner3:samplingStartDirection=sheetNormal; curve2: samplingStartDirection=sheetNormal;
    for (int curr_theta=0; curr_theta< sampling_thetaResolution; curr_theta++)
    {
        std::vector<Vertex> face_vertex_idx;
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-numTrunckU+(vResolution-2)*(sampling_thetaResolution+1)+curr_theta]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckV-numTrunckU+(vResolution-2)*(sampling_thetaResolution+1)+curr_theta+1]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-2*numCorner+(curr_theta+1)*(sampling_phiResolution+1)]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-2*numCorner+curr_theta*(sampling_phiResolution+1)]);

        resampledMesh->add_face(face_vertex_idx);
    }

    // corner3:samplingStartDirection=sheetNormal; curve3: samplingStartDirection=-sheetNormal;
    for (int curr_theta=0; curr_theta< sampling_thetaResolution; curr_theta++)
    {
        std::vector<Vertex> face_vertex_idx;
        face_vertex_idx.push_back(verticesIdx[currentTotal-2*numCorner+curr_theta*(sampling_phiResolution+1)+sampling_phiResolution]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-2*numCorner+(curr_theta+1)*(sampling_phiResolution+1)+sampling_phiResolution]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckV-numTrunckU
            +(uResolution-2)*(sampling_thetaResolution+1)+sampling_thetaResolution-curr_theta-1]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckV-numTrunckU
            +(uResolution-2)*(sampling_thetaResolution+1)+sampling_thetaResolution-curr_theta]);

        resampledMesh->add_face(face_vertex_idx);
    }

    // corner4: u=0, v=1
    // corner4:samplingStartDirection=sheetNormal; curve3: samplingStartDirection=-sheetNormal;
    for (int curr_theta=0; curr_theta< sampling_thetaResolution; curr_theta++)
    {
        std::vector<Vertex> face_vertex_idx;

        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckV-numTrunckU+sampling_thetaResolution-curr_theta]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckV-numTrunckU+sampling_thetaResolution-curr_theta-1]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-numCorner+(curr_theta+1)*(sampling_phiResolution+1)]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-numCorner+curr_theta*(sampling_phiResolution+1)]);

        resampledMesh->add_face(face_vertex_idx);
    }

    // corner4:samplingStartDirection=sheetNormal; curve4: samplingStartDirection=-sheetNormal;
    for (int curr_theta=0; curr_theta< sampling_thetaResolution; curr_theta++)
    {
        std::vector<Vertex> face_vertex_idx;
        face_vertex_idx.push_back(verticesIdx[currentTotal-numCorner+curr_theta*(sampling_phiResolution+1)+sampling_phiResolution]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-numCorner+(curr_theta+1)*(sampling_phiResolution+1)+sampling_phiResolution]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckV
            +(vResolution-2)*(sampling_thetaResolution+1)+sampling_thetaResolution-curr_theta-1]);
        face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckV
            +(vResolution-2)*(sampling_thetaResolution+1)+sampling_thetaResolution-curr_theta]);

        resampledMesh->add_face(face_vertex_idx);
    }

    // connect plane to corners, 8 faces in total
    // corner1
    std::vector<Vertex> face_vertex_idx;
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV-2*numOnePlane]);
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckV+sampling_thetaResolution]); // on curve4
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner]); // on corner1
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV]); // on curve1

    resampledMesh->add_face(face_vertex_idx);

    face_vertex_idx.clear();
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV-numOnePlane]);
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV+sampling_thetaResolution]); // on curve1
    face_vertex_idx.push_back(verticesIdx[currentTotal-3*numCorner-1]); // on corner1
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckV]); // on curve4

    resampledMesh->add_face(face_vertex_idx);

    // corner2
    face_vertex_idx.clear();
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV-2*numOnePlane+uResolution-2]);
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV+(uResolution-2)*(sampling_thetaResolution+1)]); // on curve1
    face_vertex_idx.push_back(verticesIdx[currentTotal-3*numCorner]); // on corner2
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckU-2*numTrunckV]); // on curve2

    resampledMesh->add_face(face_vertex_idx);

    face_vertex_idx.clear();
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV-numOnePlane+uResolution-2]);
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckU-2*numTrunckV+sampling_thetaResolution]); // on curve2
    face_vertex_idx.push_back(verticesIdx[currentTotal-2*numCorner-1]); // on corner2
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckU-2*numTrunckV-1]); // on curve1

    resampledMesh->add_face(face_vertex_idx);

    // corner3
    face_vertex_idx.clear();
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV-numOnePlane-1]);
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckU-2*numTrunckV+(vResolution-2)*(sampling_thetaResolution+1)]); // on curve2
    face_vertex_idx.push_back(verticesIdx[currentTotal-2*numCorner]); // on corner3
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckV-1]); // on curve3

    resampledMesh->add_face(face_vertex_idx);

    face_vertex_idx.clear();
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV-1]);
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckU-numTrunckV+(uResolution-2)*(sampling_thetaResolution+1)]); // on curve3
    face_vertex_idx.push_back(verticesIdx[currentTotal-numCorner-1]); // on corner3
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckU-numTrunckV-1]); // on curve2

    resampledMesh->add_face(face_vertex_idx);

    // corner4
    face_vertex_idx.clear();
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV-2*numOnePlane+(vResolution-2)*(uResolution-1)]);
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckU-numTrunckV+sampling_thetaResolution]); // on curve3
    face_vertex_idx.push_back(verticesIdx[currentTotal-numCorner]); // on corner4
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-1]); // on curve4

    resampledMesh->add_face(face_vertex_idx);

    face_vertex_idx.clear();
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-2*numTrunckU-2*numTrunckV-numOnePlane+(vResolution-2)*(uResolution-1)]);
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckV+(vResolution-2)*(sampling_thetaResolution+1)]); // on curve4
    face_vertex_idx.push_back(verticesIdx[currentTotal-1]); // on corner4
    face_vertex_idx.push_back(verticesIdx[currentTotal-4*numCorner-numTrunckU-numTrunckV]); // on curve3

    resampledMesh->add_face(face_vertex_idx);
}

SurfaceMeshModel *Synthesizer::blend(Node *n1, Node *n2, double t)
{
    return NULL;
}

std::vector<Array2D_Vector3> Synthesizer::planeResamping(Array2D_Vector3 mesh_faces, NURBSRectangle sheet, Vector3 initialDirection, int uResolution, int vResolution, Octree* octree)
{
    Array2D_Vector3 upsidePlane, downsidePlane;
    std::vector<Array2D_Vector3> resampledPlane;

    double delta_u = 1.0/uResolution, delta_v = 1.0/vResolution;

    for(int v_idx=1;v_idx<vResolution;v_idx++)
    {
        Vector3 sheetPoint;
        Vector3 uDirection, vDirection, sheetNormal;

        Array1D_Vector3 upIntersections, downIntersections;

        for(int u_idx=1;u_idx<uResolution;u_idx++)
        {
            sheet.GetFrame(u_idx*delta_u, v_idx*delta_v, sheetPoint, uDirection, vDirection, sheetNormal);

            if (dot(sheetNormal, initialDirection)<0)
            {
                sheetNormal=-sheetNormal;
            }

            // Up
            upIntersections.push_back( intersectionPoint( Ray(sheetPoint,sheetNormal), octree ) );

            // Down
            downIntersections.push_back( intersectionPoint( Ray(sheetPoint,-sheetNormal), octree ) );
        }
        upsidePlane.push_back(upIntersections);
        downsidePlane.push_back(downIntersections);
    }

    resampledPlane.push_back(upsidePlane);
    resampledPlane.push_back(downsidePlane);

    return resampledPlane;
}

Array2D_Vector3 Synthesizer::cylinderResampling(Array2D_Vector3 mesh_faces, NURBSCurve pathCurve, Vector3 initialDirection,
    int timeResolution, int thetaResolution, double thetaRange, Octree* octree, bool SheetBoundary)
{
    double delta_t=1.0/timeResolution;

    std::vector<Vector3> cross_section;
    Array2D_Vector3 crossSections;

    // type check
    // cylinder sampling for plane (open, thetaSamplings=theResolution+1), and sampling direction fixed as the sheet normal
    if(SheetBoundary)
    {
        for (int i=1;i<timeResolution;i++)
        {
            cross_section=resampleCoutourPointsPlane(mesh_faces,pathCurve,i*delta_t,thetaResolution, thetaRange, initialDirection, octree);
            crossSections.push_back(cross_section);
        }
    }
    // for a true cylinder (close, thetaSamplings=theResolution), and sampling direction computed as the binormal each time
    else
    {
        for (int i=1;i<timeResolution;i++)
        {
            cross_section=resampleCoutourPointsPlane(mesh_faces,pathCurve,i*delta_t,thetaResolution, thetaRange, initialDirection, octree);
            crossSections.push_back(cross_section);
        }
    }
    return crossSections;
}

Array1D_Vector3 Synthesizer::resampleCoutourPointsPlane(Array2D_Vector3 mesh_faces, NURBSCurve pathCurve,
    double curr_t, int thetaResolution, double thetaRange, Vector3 fixed_startDirection, Octree* octree)
{
    double delta_theta=thetaRange/thetaResolution;

    Array1D_Vector3 intersections;

    Vector3 current_position;
    Vector3 curveTanget; // tangent of curve
    Vector3 biNormal;
    Vector3 curveNormal;

    pathCurve.GetFrame(curr_t,current_position,curveTanget,curveNormal,biNormal);

    if (thetaRange!=2*M_PI)
    {
        thetaResolution+=1;
    }

    for (int i=0;i<thetaResolution;i++)
    {
        Vector3 intersect_point;
        double rotate_theta=i*delta_theta;

        Vector3 sampleDirection=ROTATE_VEC(sampleDirection, fixed_startDirection, rotate_theta, curveTanget);
        intersections.push_back( intersectionPoint( Ray(current_position,sampleDirection), octree ) );
    }

    return intersections;
}

Array2D_Vector3 Synthesizer::sphereResampling(Array2D_Vector3 mesh_faces, Vector3 endPoint,
    Vector3 thetaAxis, Vector3 phiAxis, int thetaResolution, int phiResolution,
    double thetaRange, double phiRange, Octree *octree)
{

    Array2D_Vector3 sphereResamplings;

    double delta_phi=phiRange/phiResolution;
    double delta_theta=thetaRange/thetaResolution;

    Vector3 startDirection;

    if (thetaRange!=2*M_PI)
    {
        thetaResolution++;
    }

    if (phiRange!=2*M_PI)
    {
        phiResolution++;
    }

    for (int curr_theta=0; curr_theta< thetaResolution; curr_theta++)
    {
        std::vector<Vector3> phiResamplings;
        double rotate_theta=curr_theta*delta_theta;

        startDirection=phiAxis;
        Vector3 sampleDirection=ROTATE_VEC(sampleDirection, startDirection, rotate_theta, thetaAxis);

        // set startDirection for phi sampling
        startDirection=sampleDirection;

        for (int curr_phi=0; curr_phi<phiResolution; curr_phi++)
        {
            double rotate_phi= curr_phi*delta_phi;

            sampleDirection = ROTATE_VEC(sampleDirection, startDirection, rotate_phi, phiAxis);
            phiResamplings.push_back( intersectionPoint( Ray(endPoint,sampleDirection), octree ) );
        }

        sphereResamplings.push_back(phiResamplings);
    }

    return sphereResamplings;
}

void Synthesizer::addCylinderFaces(Array2D_Vector3 crossSecssions, SurfaceMeshModel* mesh, std::vector<Vertex> &vertices_idx, int idxBase)
{
    int tNum=crossSecssions.size();
    int thetaNum=crossSecssions[0].size();

    for (int curr_t=0; curr_t< tNum; curr_t++)
    {
        for (int curr_theta=0; curr_theta< thetaNum; curr_theta++)
        {
            vertices_idx.push_back(mesh->add_vertex(crossSecssions[curr_t][curr_theta]));
        }
    }

    for (int curr_t= 0; curr_t<tNum-1;curr_t++)
    {
        for(int curr_theta=0; curr_theta<thetaNum-1; curr_theta++)
        {
            std::vector<Vertex> face_vertex_idx;
            face_vertex_idx.push_back(vertices_idx[curr_t*thetaNum+curr_theta+idxBase]);
            face_vertex_idx.push_back(vertices_idx[curr_t*thetaNum+curr_theta+1+idxBase]);
            face_vertex_idx.push_back(vertices_idx[(curr_t+1)*thetaNum+curr_theta+1+idxBase]);
            face_vertex_idx.push_back(vertices_idx[(curr_t+1)*thetaNum+curr_theta+idxBase]);

            mesh->add_face(face_vertex_idx);
        }
    }
}

void Synthesizer::addEndFaces(Array2D_Vector3 sphereResamplings, SurfaceMeshModel* mesh, std::vector<SurfaceMeshModel::Vertex> &vertices_idx, int idxBase)
{
    int thetaNum = sphereResamplings.size();
    int phiNum = sphereResamplings[0].size();

    for (int curr_theta=0; curr_theta< thetaNum; curr_theta++)
    {
        for (int curr_phi=0; curr_phi< phiNum; curr_phi++)
        {
            vertices_idx.push_back(mesh->add_vertex(sphereResamplings[curr_theta][curr_phi]));
        }
    }

    for (int curr_theta=0; curr_theta<thetaNum-1;curr_theta++)
    {
        for(int curr_phi=0; curr_phi<phiNum-1; curr_phi++)
        {
            std::vector<Vertex> face_vertex_idx;
            face_vertex_idx.push_back(vertices_idx[curr_theta*phiNum+curr_phi+idxBase]);
            face_vertex_idx.push_back(vertices_idx[(curr_theta+1)*phiNum+curr_phi+idxBase]);
            face_vertex_idx.push_back(vertices_idx[(curr_theta+1)*phiNum+curr_phi+1+idxBase]);
            face_vertex_idx.push_back(vertices_idx[curr_theta*phiNum+curr_phi+1+idxBase]);

            mesh->add_face(face_vertex_idx);
        }
    }

    for(int curr_theta=0; curr_theta<thetaNum-1;curr_theta++)
    {
        std::vector<Vertex> face_vertex_idx;
        face_vertex_idx.push_back(vertices_idx[curr_theta*phiNum+phiNum-1+idxBase]);
        face_vertex_idx.push_back(vertices_idx[(curr_theta+1)*phiNum+phiNum-1+idxBase]);
        face_vertex_idx.push_back(vertices_idx[(curr_theta+1)*phiNum+idxBase]);
        face_vertex_idx.push_back(vertices_idx[curr_theta*phiNum+idxBase]);

        mesh->add_face(face_vertex_idx);
    }
}

void Synthesizer::addCornerFaces(Array2D_Vector3 sphereResamplings, SurfaceMeshModel* mesh, std::vector<Vertex> &vertices_idx, int idxBase)
{
    int thetaNum=sphereResamplings.size();
    int phiNum=sphereResamplings[0].size();

    for (int curr_theta=0; curr_theta< thetaNum; curr_theta++)
    {
        for (int curr_phi=0; curr_phi< phiNum; curr_phi++)
        {
            vertices_idx.push_back(mesh->add_vertex(sphereResamplings[curr_theta][curr_phi]));
        }
    }

    for (int curr_theta=0; curr_theta<thetaNum-1;curr_theta++)
    {
        for(int curr_phi=0; curr_phi<phiNum-1; curr_phi++)
        {
            std::vector<Vertex> face_vertex_idx;
            face_vertex_idx.push_back(vertices_idx[curr_theta*phiNum+curr_phi+idxBase]);
            face_vertex_idx.push_back(vertices_idx[(curr_theta+1)*phiNum+curr_phi+idxBase]);
            face_vertex_idx.push_back(vertices_idx[(curr_theta+1)*phiNum+curr_phi+1+idxBase]);
            face_vertex_idx.push_back(vertices_idx[curr_theta*phiNum+curr_phi+1+idxBase]);

            mesh->add_face(face_vertex_idx);
        }
    }
}

void Synthesizer::addPlaneFaces(std::vector<Array2D_Vector3> resampledPlane, SurfaceMeshModel* mesh, std::vector<Vertex> &vertices_idx, int idxBase)
{
    int vNum=resampledPlane[0].size();
    int uNum=resampledPlane[0][0].size();

    for (int k=0;k<2;k++)
    {
        for(int curr_v=0; curr_v<vNum;curr_v++)
        {
            for(int curr_u=0; curr_u<uNum; curr_u++)
                vertices_idx.push_back(mesh->add_vertex(resampledPlane[k][curr_v][curr_u]));
        }
    }

    // add upside faces
    for (int curr_v= 0; curr_v<vNum-1;curr_v++)
    {
        for(int curr_u=0; curr_u<uNum-1; curr_u++)
        {
            std::vector<Vertex> face_vertex_idx;
            face_vertex_idx.push_back(vertices_idx[curr_v*uNum+curr_u+idxBase]);
            face_vertex_idx.push_back(vertices_idx[curr_v*uNum+curr_u+1+idxBase]);
            face_vertex_idx.push_back(vertices_idx[(curr_v+1)*uNum+curr_u+1+idxBase]);
            face_vertex_idx.push_back(vertices_idx[(curr_v+1)*uNum+curr_u+idxBase]);

            mesh->add_face(face_vertex_idx);
        }
    }

    // add downside faces
    for (int curr_v= 0; curr_v<vNum-1;curr_v++)
    {
        for(int curr_u=0; curr_u<uNum-1; curr_u++)
        {
            std::vector<Vertex> face_vertex_idx;
            face_vertex_idx.push_back(vertices_idx[curr_v*uNum+curr_u+idxBase+uNum*vNum]);
            face_vertex_idx.push_back(vertices_idx[(curr_v+1)*uNum+curr_u+idxBase+uNum*vNum]);
            face_vertex_idx.push_back(vertices_idx[(curr_v+1)*uNum+curr_u+1+idxBase+uNum*vNum]);
            face_vertex_idx.push_back(vertices_idx[curr_v*uNum+curr_u+1+idxBase+uNum*vNum]);

            mesh->add_face(face_vertex_idx);
        }
    }
}

Vec3d Synthesizer::intersectionPoint( Ray ray, Octree * useTree, int * faceIndex )
{
    HitResult res;
    Vec3d isetpoint(0);

    QSet<int> results = useTree->intersectRay( ray, ray.thickness, false );

    double minDistance = DBL_MAX;
    bool foundIntersection;

    foreach(int i, results)
    {
        useTree->intersectionTestOld(SurfaceMeshModel::Face(i), ray, res);

        // find the nearest intersection point
        if(res.hit)
        {
            if (res.distance < minDistance)
            {
                minDistance = res.distance;
                isetpoint = ray.origin + (ray.direction * res.distance);
                if(faceIndex) *faceIndex = i;
            }
            foundIntersection = true;
        }
    }

    assert(foundIntersection == true);

    return isetpoint;
}
