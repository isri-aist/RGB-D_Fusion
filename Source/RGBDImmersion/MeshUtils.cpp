#include "MeshUtils.h"
#include "DynamicMesh/DynamicMesh3.h"
#include "DynamicMesh/MeshNormals.h"
#include "DynamicMesh/DynamicMeshAttributeSet.h"

void RelaxLongEdges(FDynamicMesh3& Mesh, float MaxStretch = 1.5f)
{
    int32 NumIter = 1;

    for (int iter = 0; iter < NumIter; ++iter)
    {
        for (int32 tid : Mesh.TriangleIndicesItr())
        {
            UE::Geometry::FIndex3i Tri = Mesh.GetTriangle(tid);
            FVector3d P[3] = { Mesh.GetVertex(Tri.A), Mesh.GetVertex(Tri.B), Mesh.GetVertex(Tri.C) };
            
            for (int i = 0; i < 3; ++i)
            {
                int32 ia = i;
                int32 ib = (i + 1) % 3;
                int32 va = Tri[ia], vb = Tri[ib];

                double Len = (P[ia] - P[ib]).Length();
                double OrigLen = 1.0;
                
                double MaxLen = OrigLen * MaxStretch;
                if (Len > MaxLen)
                {
                    // Ramène les deux sommets au milieu, sans dépasser
                    FVector3d Mid = (P[ia] + P[ib]) * 0.5;
                    FVector3d DirA = (P[ia] - Mid).GetSafeNormal();
                    FVector3d DirB = (P[ib] - Mid).GetSafeNormal();
                    double NewHalfLen = MaxLen / 2.0;
                    Mesh.SetVertex(va, Mid + DirA * NewHalfLen);
                    Mesh.SetVertex(vb, Mid + DirB * NewHalfLen);
                }
            }
        }
    }
}

void WeldVertices(FDynamicMesh3& Mesh, double Epsilon = 1e-6)
{
    TMap<FVector3d, int32> UniqueVerts;
    TMap<int32, int32> Remap;
    for (int32 vid : Mesh.VertexIndicesItr())
    {
        FVector3d pos = Mesh.GetVertex(vid);
        int32* Existing = nullptr;
        for (auto& Pair : UniqueVerts)
        {
            if (Pair.Key.Equals(pos, Epsilon))
            {
                Existing = &Pair.Value;
                break;
            }
        }
        if (Existing)
        {
            Remap.Add(vid, *Existing);
        }
        else
        {
            UniqueVerts.Add(pos, vid);
            Remap.Add(vid, vid);
        }
    }
    // Remap triangles
    for (int32 tid : Mesh.TriangleIndicesItr())
    {
        UE::Geometry::FIndex3i tri = Mesh.GetTriangle(tid);
        tri.A = Remap[tri.A];
        tri.B = Remap[tri.B];
        tri.C = Remap[tri.C];
        Mesh.SetTriangle(tid, tri);
    }
    // Remove unused (now duplicate) vertices
    Mesh.CompactInPlace();
}
void RelaxMesh(FDynamicMesh3& Mesh, int Passes = 2, float Amount = 0.5f)
{
    TMap<int32, FVector3d> NewPositions;
    for (int pass = 0; pass < Passes; ++pass)
    {
        for (int32 vid : Mesh.VertexIndicesItr())
        {
            FVector3d Pos = Mesh.GetVertex(vid);
            FVector3d Avg = FVector3d::Zero();
            int32 N = 0;
            for (int nbr : Mesh.VtxVerticesItr(vid))
            {
                Avg += Mesh.GetVertex(nbr);
                N++;
            }
            if (N > 0)
            {
                Avg /= (double)N;
                FVector3d Relaxed = Pos * (1 - Amount) + Avg * Amount;
                NewPositions.Add(vid, Relaxed);
            }
        }
        for (auto& Pair : NewPositions)
        {
            Mesh.SetVertex(Pair.Key, Pair.Value);
        }
        NewPositions.Reset();
    }
}
UDynamicMesh* UMeshUtils::SubdivideDynamicMesh(UDynamicMesh* DynamicMesh)
{
    using namespace UE::Geometry;
    if (!DynamicMesh) return nullptr;

    DynamicMesh->EditMesh([&](FDynamicMesh3& Mesh)
    {
        auto* Attributes = Mesh.Attributes();
        auto* UVOverlay = Attributes ? Attributes->GetUVLayer(0) : nullptr;
        if (!UVOverlay) return;

        struct FTriData { int32 TriID; FIndex3i Indices; FIndex3i UVs; };
        TArray<FTriData> OriginalTris;
        for (int32 TriID : Mesh.TriangleIndicesItr())
        {
            FIndex3i Tri = Mesh.GetTriangle(TriID);
            FIndex3i UVTri = UVOverlay->GetTriangle(TriID);
            OriginalTris.Add({TriID, Tri, UVTri});
        }

        // Stockage temporaire pour positions et UVs
        TMap<int32, FVector3d> VertexPositions;
        TMap<int32, FVector2f> UVPositions;
        for (const FTriData& Data : OriginalTris)
        {
            for (int i=0; i<3; ++i)
            {
                VertexPositions.FindOrAdd(Data.Indices[i], Mesh.GetVertex(Data.Indices[i]));
                UVPositions.FindOrAdd(Data.UVs[i], UVOverlay->GetElement(Data.UVs[i]));
            }
        }

        // Supprimer tous les triangles d'abord
        for (const FTriData& TriData : OriginalTris)
            Mesh.RemoveTriangle(TriData.TriID, false);

        for (const FTriData& TriData : OriginalTris)
        {
            int32 V[3] = {TriData.Indices.A, TriData.Indices.B, TriData.Indices.C};
            int32 UV[3] = {TriData.UVs.A, TriData.UVs.B, TriData.UVs.C};

            // Calcul et ajout des milieux d'arêtes (vertex & UV, pas de partage !)
            FVector3d M01 = (VertexPositions[V[0]] + VertexPositions[V[1]]) * 0.5;
            FVector3d M12 = (VertexPositions[V[1]] + VertexPositions[V[2]]) * 0.5;
            FVector3d M20 = (VertexPositions[V[2]] + VertexPositions[V[0]]) * 0.5;
            int32 VM01 = Mesh.AppendVertex(M01);
            int32 VM12 = Mesh.AppendVertex(M12);
            int32 VM20 = Mesh.AppendVertex(M20);

            FVector2f UV01 = (UVPositions[UV[0]] + UVPositions[UV[1]]) * 0.5f;
            FVector2f UV12 = (UVPositions[UV[1]] + UVPositions[UV[2]]) * 0.5f;
            FVector2f UV20 = (UVPositions[UV[2]] + UVPositions[UV[0]]) * 0.5f;

            // Pour CHAQUE sommet de CHAQUE triangle, on crée un nouvel élément UV
            // triangle 1: V0, VM01, VM20
            int32 U0 = UVOverlay->AppendElement(UVPositions[UV[0]]);
            int32 U01 = UVOverlay->AppendElement(UV01);
            int32 U20 = UVOverlay->AppendElement(UV20);
            int32 T0 = Mesh.AppendTriangle(V[0], VM01, VM20);
            UVOverlay->SetTriangle(T0, FIndex3i(U0, U01, U20));

            // triangle 2: V1, VM12, VM01
            int32 U1 = UVOverlay->AppendElement(UVPositions[UV[1]]);
            int32 U12 = UVOverlay->AppendElement(UV12);
            int32 T1 = Mesh.AppendTriangle(V[1], VM12, VM01);
            UVOverlay->SetTriangle(T1, FIndex3i(U1, U12, U01));

            // triangle 3: V2, VM20, VM12
            int32 U2 = UVOverlay->AppendElement(UVPositions[UV[2]]);
            int32 T2 = Mesh.AppendTriangle(V[2], VM20, VM12);
            UVOverlay->SetTriangle(T2, FIndex3i(U2, U20, U12));

            // triangle 4: VM01, VM12, VM20
            int32 T3 = Mesh.AppendTriangle(VM01, VM12, VM20);
            int32 UMid = UVOverlay->AppendElement((UV01 + UV12 + UV20) / 3.0f); // barycentre
            UVOverlay->SetTriangle(T3, FIndex3i(U01, U12, U20));
        }

        // Recompute normales (vertex)
        FMeshNormals::QuickComputeVertexNormals(Mesh);
        
        if (Attributes) {
            auto* NormalOverlay = Attributes->GetNormalLayer(0);
            if (NormalOverlay) {
                // Efface l'overlay pour le remplir proprement
                NormalOverlay->ClearElements();

                // Pour chaque triangle, on crée une normale overlay par corner, copiée depuis la normale vertex
                for (int32 TriID : Mesh.TriangleIndicesItr()) {
                    FIndex3i Tri = Mesh.GetTriangle(TriID);
                    int32 N0 = NormalOverlay->AppendElement((FVector3f)Mesh.GetVertexNormal(Tri.A));
                    int32 N1 = NormalOverlay->AppendElement((FVector3f)Mesh.GetVertexNormal(Tri.B));
                    int32 N2 = NormalOverlay->AppendElement((FVector3f)Mesh.GetVertexNormal(Tri.C));
                    NormalOverlay->SetTriangle(TriID, FIndex3i(N0, N1, N2));
                }
            }
        }

        WeldVertices(Mesh);

        RelaxMesh(Mesh, 1, 0.2f); // 2 passes, 40% de lissage
    }, EDynamicMeshChangeType::GeneralEdit, EDynamicMeshAttributeChangeFlags::Unknown);

    return DynamicMesh;
}




UDynamicMesh* UMeshUtils::DisplaceMeshFromTexture(
    UDynamicMesh* DynMesh,
    UTexture2D* DisplacementTexture,
    float Magnitude,
    float NeutralGray
)
{
    using namespace UE::Geometry;
    if (!DynMesh || !DisplacementTexture)
        return DynMesh;

    if (DisplacementTexture->GetPixelFormat() != PF_R32_FLOAT)
        return DynMesh;

    auto* PlatData = DisplacementTexture->GetPlatformData();
    if (!PlatData || PlatData->Mips.Num() == 0)
        return DynMesh;

    FTexture2DMipMap& Mip = PlatData->Mips[0];
    FByteBulkData* RawImageData = &Mip.BulkData;
    if (!RawImageData)
        return DynMesh;

    float* FormattedImageData = static_cast<float*>(RawImageData->Lock(LOCK_READ_ONLY));
    if (!FormattedImageData)
        return DynMesh;

    int32 Width = Mip.SizeX;
    int32 Height = Mip.SizeY;

    // Précalcule les normales UNE fois
    TMap<int32, FVector3d> VertexNormals;
    DynMesh->EditMesh([&](FDynamicMesh3& Mesh)
    {
        FDynamicMeshUVOverlay* UVOverlay = Mesh.Attributes() ? Mesh.Attributes()->GetUVLayer(0) : nullptr;
        if (!UVOverlay) return;

        FMeshNormals::QuickComputeVertexNormals(Mesh); // 1x seulement
        for (int32 vid : Mesh.VertexIndicesItr())
            VertexNormals.Add(vid, FVector3d(Mesh.GetVertexNormal(vid)));
    });

    // Déplacement effectif : une seule passe, une seule boucle lente
    DynMesh->EditMesh([&](FDynamicMesh3& Mesh)
    {
        FDynamicMeshUVOverlay* UVOverlay = Mesh.Attributes() ? Mesh.Attributes()->GetUVLayer(0) : nullptr;
        if (!UVOverlay) return;

        for (int32 vid : Mesh.VertexIndicesItr())
        {
            int UVElement = -1;
            for (int tid : Mesh.VtxTrianglesItr(vid))
            {
                FIndex3i Tri = Mesh.GetTriangle(tid);
                for (int j = 0; j < 3; ++j)
                {
                    if (Tri[j] == vid)
                    {
                        UVElement = UVOverlay->GetTriangle(tid)[j];
                        break;
                    }
                }
                if (UVElement != -1) break;
            }
            if (UVElement == -1) continue;

            FVector2f UV = UVOverlay->GetElement(UVElement);
            float U = FMath::Clamp(UV.X, 0.0f, 1.0f);
            float V = FMath::Clamp(UV.Y, 0.0f, 1.0f);

            int X = FMath::Clamp(int(U * (Width - 1)), 0, Width - 1);
            int Y = FMath::Clamp(int(V * (Height - 1)), 0, Height - 1);
            int PixelIndex = Y * Width + X;
            if (PixelIndex < 0 || PixelIndex >= Width*Height)
                continue;

            float DisplaceValue = FormattedImageData[PixelIndex];
            float Delta = DisplaceValue - NeutralGray;

            FVector3d OldPos = Mesh.GetVertex(vid);
            FVector3d Normal = VertexNormals.Contains(vid) ? VertexNormals[vid] : FVector3d::UnitZ();
            Mesh.SetVertex(vid, OldPos + Normal * Magnitude * Delta);
        }
    });

    RawImageData->Unlock();
    return DynMesh;
}