#include "process_planning/fem_analysis.h"

#include "tetgen.h"
#include "mfem.hpp"

#include <cassert>
#include <cmath>
#include <fstream>
#include <iostream>
#include <iostream>

#include "geometry/triangular_mesh_utils.h"
#include "system/parse_stl.h"

using namespace mfem;
using namespace std;

namespace gca {

  void write_to_poly_file(const triangular_mesh& md,
			  const std::vector<index_t> faces_touching_fixed_jaw,
			  const std::vector<index_t> faces_touching_clamp_jaw,
			  const std::string& dest_file) {
    vector<index_t> vertices_touching_fixed_jaw =
      vertex_inds_on_surface(faces_touching_fixed_jaw, md);
    vector<index_t> vertices_touching_clamp_jaw =
      vertex_inds_on_surface(faces_touching_clamp_jaw, md);

    cout << "dest file = " << dest_file << endl;
    ofstream out_stream(dest_file + ".poly", std::ofstream::out);

    int num_boundary_attributes = 1;
    // vector<point> vertices;
    // vector<triangle_t> vertex_triangles =
    //   fill_vertex_triangles(md.triangles, vertices, 0.0001);

    out_stream << "# Node list" << endl;
    out_stream << md.vertex_indexes().size() << " " << 3 << " " << num_boundary_attributes << " " << 0 << endl;
    int j = 1;
    for (auto& i : md.vertex_indexes()) {
      auto t = md.vertex(i);
      out_stream << j << " " << t.x << " " << t.y << " " << t.z << " ";
      if (elem(i, vertices_touching_fixed_jaw)) {
	out_stream << 1;
      } else if (elem(i, vertices_touching_clamp_jaw)) {
	out_stream << 2;
      } else {
	out_stream << 3;
      }
      out_stream << endl;
      j++;
    }

    out_stream << endl;

    out_stream << "# Facet list" << endl;
    out_stream << md.face_indexes().size() << " " << 0 << endl;
    for (int i = 0; i < md.face_indexes().size(); i++) {
      auto t = md.triangle_vertices(i);
      out_stream << 1 << endl;
      out_stream << 3 << " ";
      out_stream << (t.v[0] + 1) << " " << (t.v[1] + 1) << " " << (t.v[2] + 1) << endl;
    }

    out_stream << endl;

    out_stream << "# Hole list" << endl;
    out_stream << 0 << endl;
    out_stream << endl;
    out_stream << "# Region list" << endl;
    out_stream << 0 << endl;
    out_stream << endl;

    out_stream.close();
  }

  void write_tetrahedral_mesh(const tetgenio& out, const std::string& out_file) {

    ofstream out_stream(out_file + ".mesh", std::ofstream::out);

    out_stream << "MFEM mesh v1.0" << endl << endl;
    out_stream << "dimension" << endl;
    out_stream << 3 << endl << endl;

    out_stream << "elements" << endl;
    out_stream << out.numberoftetrahedra << endl;

    for (int i = 0; i < out.numberoftetrahedra; i++) {
      int tet_index = 4*i;
      out_stream << 1 << " " << 4 << " " << (out.tetrahedronlist[tet_index] - 1) << " " << (out.tetrahedronlist[tet_index + 1] - 1) << " " << (out.tetrahedronlist[tet_index + 2] - 1) << " " << (out.tetrahedronlist[tet_index + 3] - 1) << endl;
    }

    out_stream << endl;

    out_stream << "boundary" << endl;
    out_stream << out.numberoftrifaces << endl;
    for (int i = 0; i < out.numberoftrifaces; i++) {
      int tri_index = 3*i;
      int v1 = out.trifacelist[tri_index] - 1;
      int v2 = out.trifacelist[tri_index + 1] - 1;
      int v3 = out.trifacelist[tri_index + 2] - 1;

      int v1_att = out.pointattributelist[v1];
      int v2_att = out.pointattributelist[v2];
      int v3_att = out.pointattributelist[v3];

      // out_stream << "# v1 = " << v1 << ", v1_att = " << v1_att << endl;
      // out_stream << "# v2 = " << v2 << ", v2_att = " << v2_att << endl;
      // out_stream << "# v3 = " << v3 << ", v3_att = " << v3_att << endl;
      if (within_eps(v1_att, 2.0) &&
	  within_eps(v1_att, v2_att) &&
	  within_eps(v1_att, v3_att)) {
	out_stream << 2 << " ";
      } else if (within_eps(v1_att, 1.0) &&
	  within_eps(v1_att, v2_att) &&
	  within_eps(v1_att, v3_att)) {
	out_stream << 1 << " ";
      } else {
	out_stream << 3 << " ";
      }
      
      out_stream << 2 << " " << (out.trifacelist[tri_index] - 1) << " " << (out.trifacelist[tri_index + 1] - 1) << " " << (out.trifacelist[tri_index + 2] - 1) << endl;
    }

    out_stream << endl;

    out_stream << "vertices" << endl;
    out_stream << out.numberofpoints << endl;
    out_stream << 3 << endl;

    cout << "# of point attributes = " << out.numberofpointattributes << endl;
    
    for (int i = 0; i < out.numberofpoints; i++) {
      int vert_index = 3*i;
      out_stream << out.pointlist[vert_index] << " " << out.pointlist[vert_index + 1] << " " << out.pointlist[vert_index + 2] << endl;
    }

    out_stream.close();
  }

  void visualize_mesh(Mesh* mesh, GridFunction& x, const bool visualization) {
    // 14. Save the displaced mesh and the inverted solution (which gives the
    //     backward displacements to the original grid). This output can be
    //     viewed later using GLVis: "glvis -m displaced.mesh -g sol.gf".
    {
      GridFunction *nodes = mesh->GetNodes();
      *nodes += x;
      x *= -1;
      ofstream mesh_ofs("displaced.mesh");
      mesh_ofs.precision(8);
      mesh->Print(mesh_ofs);
      ofstream sol_ofs("sol.gf");
      sol_ofs.precision(8);
      x.Save(sol_ofs);
    }

    // 15. Send the above data by socket to a GLVis server. Use the "n" and "b"
    //     keys in GLVis to visualize the displacements.
    if (visualization) {
      char vishost[] = "localhost";
      int  visport   = 19916;
      socketstream sol_sock(vishost, visport);
      sol_sock.precision(8);
      sol_sock << "solution\n" << *mesh << x << flush;
    }
  }

  void analyze_mesh_forces(const std::string& m_file) {
    // 1. Parse command-line options.
    const char *mesh_file = m_file.c_str();
    int order = 1;
    bool static_cond = false;
    bool visualization = 1;

    Mesh *mesh = new Mesh(mesh_file, 1, 1);
    int dim = mesh->Dimension();

    cout << "Mesh dimension = " << dim << endl;

    cout << "Attributes" << endl;   
    for (auto& att : mesh->attributes) {
      cout << att << endl;
    }
   
    cout << "Boundary attributes" << endl;   
    for (auto& att : mesh->bdr_attributes) {
      cout << att << endl;
    }

    if (mesh->attributes.Max() != 1 || mesh->bdr_attributes.Max() < 2) {
      cerr << "\nInput mesh should have one material and "
	   << "two boundary attributes! (See schematic in ex2.cpp)\n"
	   << endl;
      return;
    }

    // 3. Select the order of the finite element discretization space. For NURBS
    //    meshes, we increase the order by degree elevation.
    if (mesh->NURBSext && order > mesh->NURBSext->GetOrder()) {
      mesh->DegreeElevate(order - mesh->NURBSext->GetOrder());
    }

    // 4. Refine the mesh to increase the resolution. In this example we do
    //    'ref_levels' of uniform refinement. We choose 'ref_levels' to be the
    //    largest number that gives a final mesh with no more than 5,000
    //    elements.
    {
      int ref_levels =
	(int)floor(log(5000./mesh->GetNE())/log(2.)/dim);
      for (int l = 0; l < ref_levels; l++) {
	mesh->UniformRefinement();
      }
    }

    // 5. Define a finite element space on the mesh. Here we use vector finite
    //    elements, i.e. dim copies of a scalar finite element space. The vector
    //    dimension is specified by the last argument of the FiniteElementSpace
    //    constructor. For NURBS meshes, we use the (degree elevated) NURBS space
    //    associated with the mesh nodes.
    FiniteElementCollection *fec;
    FiniteElementSpace *fespace;

    assert(!mesh->NURBSext);

    fec = new H1_FECollection(order, dim);
    fespace = new FiniteElementSpace(mesh, fec, dim);

    cout << "Number of finite element unknowns: " << fespace->GetTrueVSize()
	 << endl << "Assembling: " << flush;

    // 6. Determine the list of true (i.e. conforming) essential boundary dofs.
    //    In this example, the boundary conditions are defined by marking only
    //    boundary attribute 1 from the mesh as essential and converting it to a
    //    list of true dofs.
    Array<int> ess_tdof_list, ess_bdr(mesh->bdr_attributes.Max());
    ess_bdr = 0;
    ess_bdr[0] = 1;
    fespace->GetEssentialTrueDofs(ess_bdr, ess_tdof_list);

    // 7. Set up the linear form b(.) which corresponds to the right-hand side of
    //    the FEM linear system. In this case, b_i equals the boundary integral
    //    of f*phi_i where f represents a "pull down" force on the Neumann part
    //    of the boundary and phi_i are the basis functions in the finite element
    //    fespace. The force is defined by the VectorArrayCoefficient object f,
    //    which is a vector of Coefficient objects. The fact that f is non-zero
    //    on boundary attribute 2 is indicated by the use of piece-wise constants
    //    coefficient for its last component.
    VectorArrayCoefficient f(dim);
    for (int i = 0; i < dim-1; i++)
      {
	f.Set(i, new ConstantCoefficient(0.0));
      }
    {
      Vector pull_force(mesh->bdr_attributes.Max());
      pull_force = 0.0;
      pull_force(1) = -1.0e-1;
      f.Set(dim-1, new PWConstCoefficient(pull_force));
    }

    LinearForm *b = new LinearForm(fespace);
    b->AddBoundaryIntegrator(new VectorBoundaryLFIntegrator(f));
    cout << "r.h.s. ... " << flush;
    b->Assemble();

    // 8. Define the solution vector x as a finite element grid function
    //    corresponding to fespace. Initialize x with initial guess of zero,
    //    which satisfies the boundary conditions.
    GridFunction x(fespace);
    x = 0.0;

    // 9. Set up the bilinear form a(.,.) on the finite element space
    //    corresponding to the linear elasticity integrator with piece-wise
    //    constants coefficient lambda and mu.
    Vector lambda(mesh->attributes.Max());
    lambda = 1.0;
    lambda(0) = lambda(1)*50;
    PWConstCoefficient lambda_func(lambda);
    Vector mu(mesh->attributes.Max());
    mu = 1.0;
    //mu(0) = mu(1)*50;
    PWConstCoefficient mu_func(mu);

    BilinearForm *a = new BilinearForm(fespace);
    a->AddDomainIntegrator(new ElasticityIntegrator(lambda_func,mu_func));

    // 10. Assemble the bilinear form and the corresponding linear system,
    //     applying any necessary transformations such as: eliminating boundary
    //     conditions, applying conforming constraints for non-conforming AMR,
    //     static condensation, etc.
    cout << "matrix ... " << flush;
    if (static_cond) { a->EnableStaticCondensation(); }
    a->Assemble();

    SparseMatrix A;
    Vector B, X;
    a->FormLinearSystem(ess_tdof_list, x, *b, A, X, B);
    cout << "done." << endl;

    cout << "Size of linear system: " << A.Height() << endl;

    // 11. Define a simple symmetric Gauss-Seidel preconditioner and use it to
    //     solve the system Ax=b with PCG.
    GSSmoother M(A);
    PCG(A, M, B, X, 1, 500, 1e-8, 0.0);

    // 12. Recover the solution as a finite element grid function.
    a->RecoverFEMSolution(X, *b, x);

    // 13. For non-NURBS meshes, make the mesh curved based on the finite element
    //     space. This means that we define the mesh elements through a fespace
    //     based transformation of the reference element. This allows us to save
    //     the displaced mesh as a curved mesh when using high-order finite
    //     element displacement field. We assume that the initial mesh (read from
    //     the file) is not higher order curved mesh compared to the chosen FE
    //     space.
    assert(!mesh->NURBSext);

    cout << "Not nurbs!" << endl;
    mesh->SetNodalFESpace(fespace);

    visualize_mesh(mesh, x, visualization);

    // 16. Free the used memory.
    delete a;
    delete b;
    if (fec)
      {
	delete fespace;
	delete fec;
      }
    delete mesh;
  }

  //int analyze(const std::string& stl_file) {//(int argc, char *argv[]) {
  int analyze(const triangular_mesh& m,
	      const std::vector<index_t> faces_touching_fixed_jaw,
	      const std::vector<index_t> faces_touching_clamp_jaw) {

    // string target_file(argv[1]);
    // triangular_mesh mesh_data = parse_stl(target_file, 0.001);
    string mesh_file = "meshfile";
    //write_to_poly_file(mesh_data, mesh_file);
    write_to_poly_file(m,
		       faces_touching_fixed_jaw,
		       faces_touching_clamp_jaw,
		       mesh_file);

    tetgenbehavior b;

    //b.parse_commandline(2, argv);
    b.plc = true;
    b.refine = false;
    string infile = mesh_file;
    for (int i = 0; i < infile.size(); i++) {
      b.infilename[i] = infile[i];
    }
    b.infilename[infile.size()] = '\0';
  
    b.object = tetgenbehavior::POLY;

    strcpy(b.outfilename, b.infilename);
    strcat(b.outfilename, ".1");

    tetgenio in, addin, bgmin;
  
    assert(!b.refine);
    assert(b.plc);
    if (!in.load_plc(b.infilename, (int) b.object)) {
      terminatetetgen(NULL, 10);
    }

    tetgenio out;

    tetrahedralize(&b, &in, &out, &addin, &bgmin);
    //tetrahedralize(&b, &in, NULL, &addin, &bgmin);

    string tetfile = "output_mesh";
    write_tetrahedral_mesh(out, tetfile);

    analyze_mesh_forces(tetfile + ".mesh");
    return 0;
  }
  
}
