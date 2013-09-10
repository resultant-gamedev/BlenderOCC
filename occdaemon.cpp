/**
 * soppermann@acm.org
 * 1-6-2008
 */
/**
 * quick & dirty get tris from occ
 */
#include "vect.h"

#include <boost/interprocess/managed_shared_memory.hpp>
#include <boost/interprocess/containers/vector.hpp>
#include <boost/interprocess/allocators/allocator.hpp>
#include <string>
#include <vector>

#include <cstdlib>

#include <config.h>
#include <Standard_Type.hxx>
//#include <igesread.h>
#include <Standard.hxx>
#include <Standard_Boolean.hxx>

#include <IGESControl_Controller.hxx>
#include <IGESControl_Reader.hxx>
#include <Message_MsgFile.hxx>
#include <TopoDS.hxx>
#include <TColStd_HSequenceOfTransient.hxx>
//#include <TopoDS_Shape.hxx>
#include <TopoDS_Shape.hxx>
#include <TopoDS_Edge.hxx>
#include <TColStd_HSequenceOfTransient.hxx>
#include <STEPControl_Reader.hxx>
#include <IGESControl_Reader.hxx>
//#include <TopoDS_Shape.hxx>
#include <BRepTools.hxx>
#include <BRepMesh.hxx>
#include <BRepMesh_IncrementalMesh.hxx>
#include <Bnd_Box.hxx>
#include <BRepBndLib.hxx>
#include <BRepTools.hxx>
#include <TopoDS_Iterator.hxx>
#include <TopExp.hxx>
#include <Poly_Triangulation.hxx>
#include <Poly_Array1OfTriangle.hxx>
#include <BRep_Tool.hxx>
#include <TopExp_Explorer.hxx>


typedef boost::interprocess::allocator<Vector3Df, boost::interprocess::managed_shared_memory::segment_manager>  Vector3DfAllocator;
typedef boost::interprocess::vector<Vector3Df, Vector3DfAllocator> TestFace;
typedef boost::interprocess::allocator<TestFace, boost::interprocess::managed_shared_memory::segment_manager> TestFaceAllocator;
typedef boost::interprocess::vector<TestFace, TestFaceAllocator> TestFaces;

void loadIGES(char* filename)
{
    if(!filename) return;
    std::cout << "test" << std::endl;

    boost::interprocess::managed_shared_memory segment(boost::interprocess::open_only, "OCCSharedMem");  
    TestFaces *m_trisp = segment.find<TestFaces>("m_tris").first;
    //TestFaces *m_facesp = segment.find<TestFaces>("m_faces").first;

    Vector3DfAllocator vector3df_alloc_inst(segment.get_segment_manager());
    TestFace *m_facep = segment.construct<TestFace>("m_facep")(vector3df_alloc_inst);

    std::cout << "connected to shared mem" << std::endl;
    std::cout << filename << std::endl;

    //STEPControl_Reader *readerp = new STEPControl_Reader;
    IGESControl_Controller::Init();
    Message_MsgFile::LoadFromEnv("CSF_XSMessage","IGES");
    Message_MsgFile::LoadFromEnv("CSF_SHMessageStd","SHAPEStd");
    IGESControl_Reader reader;
    std::cout << filename << std::endl;
    reader.ReadFile(filename);
    reader.PrintCheckLoad(Standard_True,IFSelect_GeneralInfo);

    Standard_Integer NbRoots = reader.NbRootsForTransfer();
    std::cout << "Number of Roots in the IGES File: " << NbRoots << std::endl;
    Standard_Integer NbTrans = reader.TransferRoots();
    std::cout << "IGES roots transferred: " << NbTrans << std::endl;
    std::cout << "Number of resulting shapes is: " << reader.NbShapes() << std::endl;
    TopoDS_Shape resulting_shape = reader.OneShape();
    // gp_Trsf theTrans;
    // gp_Axl Axis = gp_Axl(gp_Pnt(200,60,60),gp_Dir(0.,1.,0.)); 
    // theTrans.SetRotation(Axis,30*PI/180); // Rotation of 30 degrees 
    // BRepBuilderAPI_Transform myBRepTransformation(resulting_shape,theTrans,true); 
    // TopoDS_Shape TransformedShape = myBRepTransformation.Shape();
    TopoDS_Iterator topo_iter;
    //m_topodsshapes.push_back(resulting_shape);
    //BRepMesh::Mesh(resulting_shape, 1.0);
    TopExp_Explorer faceExp(resulting_shape, TopAbs_FACE);

    for(; faceExp.More(); faceExp.Next())
    {


        //  TopExp_Explorer vertexExp(faceExp.Current(), TopAbs_VERTEX);
        //int f_n = 0;
        TopLoc_Location L = faceExp.Current().Location();
        BRepMesh::Mesh(faceExp.Current(), .1);
        Handle (Poly_Triangulation) facing = BRep_Tool::Triangulation(TopoDS::Face(faceExp.Current()),L);
        //  const Poly_Array1OfTriangle & triangles = facing->Triangles();
        //  const TColgp_Array1OfPnt & nodes = facing->Nodes();
        //  std::cout << "opencascaded: facing->NbTriangles() = " << facing->NbTriangles() << std::endl;
        if (!facing.IsNull())
        {
            TopExp_Explorer vertexExp(faceExp.Current(), TopAbs_VERTEX);
            //int f_n = 0;
            //TopLoc_Location L = faceExp.Current().Location();
            // BRepMesh::Mesh(faceExp.Current(), .1);
            // Handle (Poly_Triangulation) facing = BRep_Tool::Triangulation(TopoDS::Face(faceExp.Current()),L);
            const Poly_Array1OfTriangle & triangles = facing->Triangles();
            const TColgp_Array1OfPnt & nodes = facing->Nodes();
            std::cout << "opencascaded: facing->NbTriangles() = " << facing->NbTriangles() << std::endl;

            for ( int i=facing->NbTriangles(); i >= 1; --i )
            {
                m_facep->clear();
                Poly_Triangle triangle = triangles(i);

                Standard_Integer node1,node2,node3;
                triangle.Get(node1, node2, node3);

                gp_Pnt v1 = nodes(node1).Transformed(L);
                gp_Pnt v2 = nodes(node2).Transformed(L);
                gp_Pnt v3 = nodes(node3).Transformed(L);

                m_facep->push_back(Vector3Df(v1.X(), v1.Y(), v1.Z()));
                m_facep->push_back(Vector3Df(v2.X(), v2.Y(), v2.Z()));
                m_facep->push_back(Vector3Df(v3.X(), v3.Y(), v3.Z()));
                m_trisp->push_back(*m_facep);

            }
        }

    }

}

void loadSTEP(char* filename)
{
    if(!filename) return;
    std::cout << "test" << std::endl;
    boost::interprocess::managed_shared_memory segment(boost::interprocess::open_only, "OCCSharedMem");  
    TestFaces *m_trisp = segment.find<TestFaces>("m_tris").first;
    //TestFaces *m_facesp = segment.find<TestFaces>("m_faces").first;

    Vector3DfAllocator vector3df_alloc_inst(segment.get_segment_manager());
    TestFace *m_facep = segment.construct<TestFace>("m_facep")(vector3df_alloc_inst);

    std::cout << "connected to shared mem" << std::endl;
    std::cout << filename << std::endl;

    //STEPControl_Reader *readerp = new STEPControl_Reader;
    STEPControl_Reader reader;
    std::cout << filename << std::endl;
    reader.ReadFile(filename);
    Standard_Integer NbRoots = reader.NbRootsForTransfer();
    std::cout << "Number of Roots in the STEP File: " << NbRoots << std::endl;
    Standard_Integer NbTrans = reader.TransferRoots();
    std::cout << "STEP roots transferred: " << NbTrans << std::endl;
    std::cout << "Number of resulting shapes is: " << reader.NbShapes() << std::endl;
    TopoDS_Shape resulting_shape = reader.OneShape();
    // gp_Trsf theTrans;
    // gp_Axl Axis = gp_Axl(gp_Pnt(200,60,60),gp_Dir(0.,1.,0.)); 
    // theTrans.SetRotation(Axis,30*PI/180); // Rotation of 30 degrees 
    // BRepBuilderAPI_Transform myBRepTransformation(resulting_shape,theTrans,true); 
    // TopoDS_Shape TransformedShape = myBRepTransformation.Shape();
    TopoDS_Iterator topo_iter;
    //m_topodsshapes.push_back(resulting_shape);
    // BRepMesh::Mesh(resulting_shape, 0.05);
    TopExp_Explorer faceExp(resulting_shape, TopAbs_FACE);

    for(; faceExp.More(); faceExp.Next())
    {


        //  TopExp_Explorer vertexExp(faceExp.Current(), TopAbs_VERTEX);
        //int f_n = 0;
        TopLoc_Location L = faceExp.Current().Location();
        BRepMesh::Mesh(faceExp.Current(), .1);
        Handle (Poly_Triangulation) facing = BRep_Tool::Triangulation(TopoDS::Face(faceExp.Current()),L);
        //  const Poly_Array1OfTriangle & triangles = facing->Triangles();
        //  const TColgp_Array1OfPnt & nodes = facing->Nodes();
        //  std::cout << "opencascaded: facing->NbTriangles() = " << facing->NbTriangles() << std::endl;
        if (!facing.IsNull())
        {
            TopExp_Explorer vertexExp(faceExp.Current(), TopAbs_VERTEX);
            //int f_n = 0;
            //TopLoc_Location L = faceExp.Current().Location();
            // BRepMesh::Mesh(faceExp.Current(), .1);
            // Handle (Poly_Triangulation) facing = BRep_Tool::Triangulation(TopoDS::Face(faceExp.Current()),L);
            const Poly_Array1OfTriangle & triangles = facing->Triangles();
            const TColgp_Array1OfPnt & nodes = facing->Nodes();
            std::cout << "opencascaded: facing->NbTriangles() = " << facing->NbTriangles() << std::endl;

            for ( int i=facing->NbTriangles(); i >= 1; --i )
            {
                m_facep->clear();
                Poly_Triangle triangle = triangles(i);

                Standard_Integer node1,node2,node3;
                triangle.Get(node1, node2, node3);

                gp_Pnt v1 = nodes(node1).Transformed(L);
                gp_Pnt v2 = nodes(node2).Transformed(L);
                gp_Pnt v3 = nodes(node3).Transformed(L);

                m_facep->push_back(Vector3Df(v1.X(), v1.Y(), v1.Z()));
                m_facep->push_back(Vector3Df(v2.X(), v2.Y(), v2.Z()));
                m_facep->push_back(Vector3Df(v3.X(), v3.Y(), v3.Z()));
                m_trisp->push_back(*m_facep);

            }
        }
        /*
           for(; vertexExp.More(); vertexExp.Next())
           {
           m_facep->clear();
           const TopoDS_Vertex& aVertex = TopoDS::Vertex(vertexExp.Current());
        //TopoDS_Edge edge = TopoDS::Edge(shape);
        TopLoc_Location location;
        Standard_Real pFirst, pLast;
        //Handle(Geom_Curve) curve = BRep_Tool::Curve(anEdge, location, pFirst, pLast);
        //std::cout << "pFirst = " << pFirst << std::endl;
        gp_Pnt p = BRep_Tool::Pnt(aVertex);
        std::cout << "Vector3Df(" << p.X() << ", " << p.Y() << ", " << p.Z() << std::endl;
        //points.push_back(Vector3Df(p.X()/1000.0, p.Y()/1000.0, p.Z()/1000.0));
        m_facep->push_back(Vector3Df(p.X()/1000.0, p.Y()/1000.0, p.Z()/1000.0));
        //      TopLoc_Location L;
        //        Handle (Poly_Triangulation) facing = BRep_Tool::Triangulation(aFace,L);
        //        if(!(facing.IsNull()))std::cout << "Number of Triangles in Face #" << f_n << ": " << facing->NbTriangles() << std::endl;
        //        f_n++;
        }
        m_facesp->push_back(*m_facep);
        */
    }
    //std::cout << " opencascaded: m_trisp->size()" << m_trisp->size() << std::endl;

    //  segment.destroy<TestFaces>("m_tris");

}

int main(int argc, char *argv[])
{
    try
    {
        if(std::string(argv[2]) == std::string("STEP"))
        {
            loadSTEP(argv[1]);
            std::cout << "load step" << std::endl;
        }else if(std::string(argv[2]) == std::string("IGES"))
        {
            loadIGES(argv[1]);
            std::cout << "load iges" << std::endl;
        }
    }
    catch(...)
    {
        std::cerr << "Unknown Exception in opencascadedaemon occured!" << std::endl;
    }

    return 0;
};
