//-----------------------------------------------------------------------------
// The 2d vector output stuff that isn't specific to any particular file
// format: getting the appropriate lines and curves, performing hidden line
// removal, calculating bounding boxes, and so on. Also raster and triangle
// mesh output.
//
// Copyright 2008-2013 Jonathan Westhues.
//-----------------------------------------------------------------------------
#include "solvespace.h"
#include "config.h"
#include <ctime>

namespace SolveSpace {

void SolveSpaceUI::ExportSectionTo(const Platform::Path &filename) {
    Vector gn = (SS.GW.projRight).Cross(SS.GW.projUp);
    gn = gn.WithMagnitude(1);

    Group *g = SK.GetGroup(SS.GW.activeGroup);
    g->GenerateDisplayItems();
    if(g->displayMesh.IsEmpty()) {
        Error(_("No solid model present; draw one with extrudes and revolves, "
                "or use Export 2d View to export bare lines and curves."));
        return;
    }

    // The plane in which the exported section lies; need this because we'll
    // reorient from that plane into the xy plane before exporting.
    Vector origin, u, v, n;
    double d;

    SS.GW.GroupSelection();
    auto const &gs = SS.GW.gs;
    if((gs.n == 0 && g->activeWorkplane != Entity::FREE_IN_3D)) {
        Entity *wrkpl = SK.GetEntity(g->activeWorkplane);
        origin = wrkpl->WorkplaneGetOffset();
        n = wrkpl->Normal()->NormalN();
        u = wrkpl->Normal()->NormalU();
        v = wrkpl->Normal()->NormalV();
    } else if(gs.n == 1 && gs.faces == 1) {
        Entity *face = SK.GetEntity(gs.entity[0]);
        origin = face->FaceGetPointNum();
        n = face->FaceGetNormalNum();
        if(n.Dot(gn) < 0) n = n.ScaledBy(-1);
        u = n.Normal(0);
        v = n.Normal(1);
    } else if(gs.n == 3 && gs.vectors == 2 && gs.points == 1) {
        Vector ut = SK.GetEntity(gs.entity[0])->VectorGetNum(),
               vt = SK.GetEntity(gs.entity[1])->VectorGetNum();
        ut = ut.WithMagnitude(1);
        vt = vt.WithMagnitude(1);

        if(fabs(SS.GW.projUp.Dot(vt)) < fabs(SS.GW.projUp.Dot(ut))) {
            swap(ut, vt);
        }
        if(SS.GW.projRight.Dot(ut) < 0) ut = ut.ScaledBy(-1);
        if(SS.GW.projUp.   Dot(vt) < 0) vt = vt.ScaledBy(-1);

        origin = SK.GetEntity(gs.point[0])->PointGetNum();
        n = ut.Cross(vt);
        u = ut.WithMagnitude(1);
        v = (n.Cross(u)).WithMagnitude(1);
    } else {
        Error(_("Bad selection for export section. Please select:\n\n"
                "    * nothing, with an active workplane "
                          "(workplane is section plane)\n"
                "    * a face (section plane through face)\n"
                "    * a point and two line segments "
                          "(plane through point and parallel to lines)\n"));
        return;
    }
    SS.GW.ClearSelection();

    n = n.WithMagnitude(1);
    d = origin.Dot(n);

    SEdgeList el = {};
    SBezierList bl = {};

    // If there's a mesh, then grab the edges from it.
    g->runningMesh.MakeEdgesInPlaneInto(&el, n, d);

    // If there's a shell, then grab the edges and possibly Beziers.
    bool export_as_pwl = SS.exportPwlCurves || fabs(SS.exportOffset) > LENGTH_EPS;
    g->runningShell.MakeSectionEdgesInto(n, d, &el, export_as_pwl ? NULL : &bl);

    // All of these are solid model edges, so use the appropriate style.
    SEdge *se;
    for(se = el.l.First(); se; se = el.l.NextAfter(se)) {
        se->auxA = Style::SOLID_EDGE;
    }
    SBezier *sb;
    for(sb = bl.l.First(); sb; sb = bl.l.NextAfter(sb)) {
        sb->auxA = Style::SOLID_EDGE;
    }

    // Remove all overlapping edges/beziers to merge the areas they describe.
    el.CullExtraneousEdges(/*both=*/true);
    bl.CullIdenticalBeziers(/*both=*/true);

    // Collect lines and beziers with custom style & export.
    for(auto &ent : SK.entity) {
        Entity *e = &ent;
        if (!e->IsVisible()) continue;
        if (e->style.v < Style::FIRST_CUSTOM) continue;
        if (!Style::Exportable(e->style.v)) continue;
        if (!e->IsInPlane(n,d)) continue;
        if (export_as_pwl) {
            e->GenerateEdges(&el);
        } else {
            e->GenerateBezierCurves(&bl);
        }
    }

    // Only remove half of the overlapping edges/beziers to support TTF Stick Fonts.
    el.CullExtraneousEdges(/*both=*/false);
    bl.CullIdenticalBeziers(/*both=*/false);

    // And write the edges.
    VectorFileWriter *out = VectorFileWriter::ForFile(filename);
    if(out) {
        // parallel projection (no perspective), and no mesh
        ExportLinesAndMesh(&el, &bl, NULL,
                           u, v, n, origin, 0,
                           out);
    }
    el.Clear();
    bl.Clear();
}

// This is an awful temporary hack to replace Constraint::GetEdges until we have proper
// export through Canvas.
class GetEdgesCanvas : public Canvas {
public:
    Camera     camera;
    SEdgeList *edges;

    const Camera &GetCamera() const override {
        return camera;
    }

    void DrawLine(const Vector &a, const Vector &b, hStroke hcs) override {
        edges->AddEdge(a, b, Style::CONSTRAINT);
    }
    void DrawEdges(const SEdgeList &el, hStroke hcs) override {
        for(const SEdge &e : el.l) {
            edges->AddEdge(e.a, e.b, Style::CONSTRAINT);
        }
    }
    void DrawVectorText(const std::string &text, double height,
                        const Vector &o, const Vector &u, const Vector &v,
                        hStroke hcs) override {
        auto traceEdge = [&](Vector a, Vector b) { edges->AddEdge(a, b, Style::CONSTRAINT); };
        VectorFont::Builtin()->Trace(height, o, u, v, text, traceEdge, camera);
    }

    void DrawQuad(const Vector &a, const Vector &b, const Vector &c, const Vector &d,
                  hFill hcf) override {
        // Do nothing
    }

    bool DrawBeziers(const SBezierList &bl, hStroke hcs) override {
        ssassert(false, "Not implemented");
    }
    void DrawOutlines(const SOutlineList &ol, hStroke hcs, DrawOutlinesAs drawAs) override {
        ssassert(false, "Not implemented");
    }
    void DrawPoint(const Vector &o, hStroke hcs) override {
        ssassert(false, "Not implemented");
    }
    void DrawPolygon(const SPolygon &p, hFill hcf) override {
        ssassert(false, "Not implemented");
    }
    void DrawMesh(const SMesh &m, hFill hcfFront, hFill hcfBack = {}) override {
        ssassert(false, "Not implemented");
    }
    void DrawFaces(const SMesh &m, const std::vector<uint32_t> &faces, hFill hcf) override {
        ssassert(false, "Not implemented");
    }
    void DrawPixmap(std::shared_ptr<const Pixmap> pm,
                            const Vector &o, const Vector &u, const Vector &v,
                            const Point2d &ta, const Point2d &tb, hFill hcf) override {
        ssassert(false, "Not implemented");
    }
    void InvalidatePixmap(std::shared_ptr<const Pixmap> pm) override {
        ssassert(false, "Not implemented");
    }
};

void SolveSpaceUI::ExportViewOrWireframeTo(const Platform::Path &filename, bool exportWireframe) {
    SEdgeList edges = {};
    SBezierList beziers = {};

    VectorFileWriter *out = VectorFileWriter::ForFile(filename);
    if(!out) return;

    SS.exportMode = true;
    GenerateAll(Generate::ALL);

    SMesh *sm = NULL;
    if(SS.GW.showShaded || SS.GW.drawOccludedAs != GraphicsWindow::DrawOccludedAs::VISIBLE) {
        Group *g = SK.GetGroup(SS.GW.activeGroup);
        g->GenerateDisplayItems();
        sm = &(g->displayMesh);
    }
    if(sm && sm->IsEmpty()) {
        sm = NULL;
    }

    for(auto &entity : SK.entity) {
        Entity *e = &entity;
        if(!e->IsVisible()) continue;

        if(SS.exportPwlCurves || sm || fabs(SS.exportOffset) > LENGTH_EPS)
        {
            // We will be doing hidden line removal, which we can't do on
            // exact curves; so we need things broken down to pwls. Same
            // problem with cutter radius compensation.
            e->GenerateEdges(&edges);
        } else {
            e->GenerateBezierCurves(&beziers);
        }
    }

    if(SS.GW.showEdges || SS.GW.showOutlines) {
        Group *g = SK.GetGroup(SS.GW.activeGroup);
        g->GenerateDisplayItems();
        if(SS.GW.showEdges) {
            g->displayOutlines.ListTaggedInto(&edges, Style::SOLID_EDGE);
        }
    }

    if(SS.GW.showConstraints != GraphicsWindow::ShowConstraintMode::SCM_NOSHOW ) {
        if(!out->OutputConstraints(&SK.constraint)) {
            GetEdgesCanvas canvas = {};
            canvas.camera = SS.GW.GetCamera();
            canvas.edges  = &edges;

            // The output format cannot represent constraints directly,
            // so convert them to edges.
            for(Constraint &c : SK.constraint) {
                c.Draw(Constraint::DrawAs::DEFAULT, &canvas);
            }

            canvas.Clear();
        }
    }

    if(exportWireframe) {
        Vector u = {1.0, 0.0, 0.0},
               v = {0.0, 1.0, 0.0},
               n = {0.0, 0.0, 1.0},
               origin = {};
        double cameraTan = 0.0,
               scale = 1.0;

        out->SetModelviewProjection(u, v, n, origin, cameraTan, scale);

        ExportWireframeCurves(&edges, &beziers, out);
    } else {
        Vector u = SS.GW.projRight,
               v = SS.GW.projUp,
               n = u.Cross(v),
               origin = SS.GW.offset.ScaledBy(-1);

        out->SetModelviewProjection(u, v, n, origin,
                                    SS.CameraTangent()*SS.GW.scale, SS.exportScale);

        ExportLinesAndMesh(&edges, &beziers, sm,
                           u, v, n, origin, SS.CameraTangent()*SS.GW.scale,
                           out);

        if(!out->HasCanvasSize()) {
            // These file formats don't have a canvas size, so they just
            // get exported in the raw coordinate system. So indicate what
            // that was on-screen.
            SS.justExportedInfo.showOrigin = true;
            SS.justExportedInfo.pt = origin;
            SS.justExportedInfo.u = u;
            SS.justExportedInfo.v = v;
        } else {
            SS.justExportedInfo.showOrigin = false;
        }

        SS.justExportedInfo.draw = true;
        GW.Invalidate();
    }

    edges.Clear();
    beziers.Clear();
}

void SolveSpaceUI::ExportWireframeCurves(SEdgeList *sel, SBezierList *sbl,
                           VectorFileWriter *out)
{
    SBezierLoopSetSet sblss = {};
    SEdge *se;
    for(se = sel->l.First(); se; se = sel->l.NextAfter(se)) {
        SBezier sb = SBezier::From(
                                (se->a).ScaledBy(1.0 / SS.exportScale),
                                (se->b).ScaledBy(1.0 / SS.exportScale));
        sblss.AddOpenPath(&sb);
    }

    sbl->ScaleSelfBy(1.0/SS.exportScale);
    SBezier *sb;
    for(sb = sbl->l.First(); sb; sb = sbl->l.NextAfter(sb)) {
        sblss.AddOpenPath(sb);
    }

    out->OutputLinesAndMesh(&sblss, NULL);
    sblss.Clear();
}

//-----------------------------------------------------------------------------
// Engineering-drawing export (phase 1): render a fixed set of standard views
// (front/top/right/iso) of the model onto a single A4 sheet, laid out in a
// grid at one common scale, as hidden-line views. Reference dimensions that
// the user added in the model are included. No title block yet.
//-----------------------------------------------------------------------------
namespace {
// A VectorFileWriter that captures the projected 2D geometry handed to it
// instead of writing a file, so several views can be repositioned onto one
// sheet at a common scale.
class CapturedView : public VectorFileWriter {
public:
    struct Stroke {
        RgbaColor            strokeRgb;
        double               lineWidth;
        bool                 filled;
        RgbaColor            fillRgb;
        hStyle               hs;
        std::vector<SBezier> beziers;
    };
    std::vector<Stroke> strokes;
    Stroke             *cur = nullptr;

    void StartPath(RgbaColor strokeRgb, double lineWidth, bool filled,
                   RgbaColor fillRgb, hStyle hs) override {
        strokes.push_back({ strokeRgb, lineWidth, filled, fillRgb, hs, {} });
        cur = &strokes.back();
    }
    void FinishPath(RgbaColor, double, bool, RgbaColor, hStyle) override { cur = nullptr; }
    void Bezier(SBezier *sb) override { if(cur) cur->beziers.push_back(*sb); }
    void Triangle(STriangle *) override {}
    void Background(RgbaColor) override {}
    void StartFile() override {}
    void FinishAndCloseFile() override {}
    bool HasCanvasSize() const override { return false; }
    bool CanOutputMesh() const override { return false; }
};
} // namespace

void SolveSpaceUI::DrawingPaperSize(int i, const char **name, double *w, double *h) {
    static const struct { const char *n; double w, h; } table[kNumPaperSizes] = {
        { "A5",      148.0, 210.0   },
        { "A4",      210.0, 297.0   },
        { "A3",      297.0, 420.0   },
        { "Letter",  215.9, 279.4   },
        { "Legal",   215.9, 355.6   },
        { "Tabloid", 279.4, 431.8   },
    };
    if(i < 0 || i >= kNumPaperSizes) i = 1; // A4
    if(name) *name = table[i].n;
    if(w)    *w    = table[i].w;
    if(h)    *h    = table[i].h;
}

const char *SolveSpaceUI::DrawingViewName(int i) {
    static const char *names[kNumDrawingViews] = {
        "FRONT", "BACK", "LEFT", "RIGHT", "TOP", "BOTTOM", "ISO", "CURRENT"
    };
    if(i < 0 || i >= kNumDrawingViews) return "";
    return names[i];
}

// Truncate a string with an ellipsis so it fits within maxW at cap height h.
static std::string FitText(const std::string &s, double maxW, double h) {
    if(s.empty() || VectorFont::Builtin()->GetWidth(h, s) <= maxW) return s;
    std::string t = s;
    while(!t.empty()) {
        t.pop_back();
        if(VectorFont::Builtin()->GetWidth(h, t + "...") <= maxW) return t + "...";
    }
    return "";
}

// Append a standard bottom-right title block (box + labelled fields, drawn as
// linework) to a sheet item, in page mm.
static void AddTitleBlock(DrawingSheet::Item *item, double L, double B, double W, double H,
                          const std::string &title, const std::string &name,
                          const std::string &scaleStr, const std::string &units,
                          const std::string &date, const std::string &drawnBy,
                          const std::string &rev, int page, int nPages) {
    RgbaColor black = RGBi(0, 0, 0);
    hStyle    hcs   = { Style::CONSTRAINT };

    DrawingSheet::Stroke lines;
    lines.strokeRgb = black; lines.lineWidth = max(0.2, SS.drawingLineWidth);
    lines.filled = false; lines.fillRgb = black; lines.hs = hcs;
    auto line = [&](double x1, double y1, double x2, double y2) {
        lines.beziers.push_back(SBezier::From(Vector::From(x1, y1, 0), Vector::From(x2, y2, 0)));
    };

    DrawingSheet::Stroke text;
    text.strokeRgb = black; text.lineWidth = 0.2;
    text.filled = false; text.fillRgb = black; text.hs = hcs;
    auto addText = [&](const std::string &s, double x, double y, double h) {
        VectorFont::Builtin()->Trace(h, Vector::From(x, y, 0),
            Vector::From(1, 0, 0), Vector::From(0, 1, 0), s,
            [&](Vector a, Vector b) { text.beziers.push_back(SBezier::From(a, b)); });
    };

    double T = B + H, R = L + W, pad = 1.5;
    double midY = B + H * 0.5;  // divider: title on top, fields below

    line(L, B, R, B); line(R, B, R, T); line(R, T, L, T); line(L, T, L, B);
    line(L, midY, R, midY);

    // Title row: title at left, "BY" at right; truncate the title to the gap.
    std::string by  = drawnBy.empty() ? "" : ("BY: " + drawnBy);
    double      byW = by.empty() ? 0.0 : VectorFont::Builtin()->GetWidth(2.0, by);
    std::string ttl = FitText(title.empty() ? "(untitled)" : title,
                              W - 3 * pad - byW, 2.8);
    addText(ttl, L + pad, midY + (T - midY - 2.8) / 2, 2.8);
    if(!by.empty()) addText(by, R - pad - byW, midY + (T - midY - 2.0) / 2, 2.0);

    // Field row, weighted so the name gets the most room; values truncated to
    // their cell so long filenames can't bleed into the next field.
    struct Field { const char *label; std::string value; double weight; };
    Field fields[] = {
        { "DWG",   name,                                2.8  },
        { "SCALE", scaleStr,                            0.9  },
        { "UNITS", units,                               0.55 },
        { "REV",   rev,                                 0.55 },
        { "DATE",  date,                                1.2  },
        { "SHEET", ssprintf("%d/%d", page + 1, nPages), 0.8  },
    };
    double totalW = 0;
    for(auto &f : fields) totalW += f.weight;
    double x = L;
    bool first = true;
    for(auto &f : fields) {
        double cwi = W * (f.weight / totalW);
        if(!first) line(x, B, x, midY);
        first = false;
        double avail = cwi - 2 * pad;
        addText(f.label,                       x + pad, midY - 2.0, 1.4);
        addText(FitText(f.value, avail, 2.2),  x + pad, B + 1.2,    2.2);
        x += cwi;
    }

    item->strokes.push_back(lines);
    item->strokes.push_back(text);
}

// An empty bordered NOTES box, filling the space to the left of the title
// block (a spot for hand-written notes after printing).
static void AddNotesBox(DrawingSheet::Item *item, double L, double B, double W, double H) {
    RgbaColor black = RGBi(0, 0, 0);
    hStyle    hcs   = { Style::CONSTRAINT };

    DrawingSheet::Stroke lines;
    lines.strokeRgb = black; lines.lineWidth = max(0.2, SS.drawingLineWidth);
    lines.filled = false; lines.fillRgb = black; lines.hs = hcs;
    double T = B + H, R = L + W;
    auto line = [&](double x1, double y1, double x2, double y2) {
        lines.beziers.push_back(SBezier::From(Vector::From(x1, y1, 0), Vector::From(x2, y2, 0)));
    };
    line(L, B, R, B); line(R, B, R, T); line(R, T, L, T); line(L, T, L, B);
    item->strokes.push_back(lines);

    DrawingSheet::Stroke text;
    text.strokeRgb = black; text.lineWidth = 0.2;
    text.filled = false; text.fillRgb = black; text.hs = hcs;
    VectorFont::Builtin()->Trace(1.6, Vector::From(L + 1.5, T - 2.4, 0),
        Vector::From(1, 0, 0), Vector::From(0, 1, 0), "NOTES",
        [&](Vector a, Vector b) { text.beziers.push_back(SBezier::From(a, b)); });
    item->strokes.push_back(text);
}

void SolveSpaceUI::ComputeDrawingSheet(DrawingSheet *sheet, bool allowMultiPage) {
    sheet->Clear();

    // Save state we temporarily change to capture geometry in model mm.
    Vector savedRight = SS.GW.projRight, savedUp = SS.GW.projUp;
    double savedScale = SS.exportScale, savedOffset = SS.exportOffset;
    double savedGwScale = SS.GW.scale;

    // Standard view orientations as (right, up); n = right x up faces the
    // viewer. Order matches DrawingViewName()/the drawingViewMask bits.
    struct ViewDef { const char *name; Vector u, v; };
    Vector isoU = Vector::From(1, 0, -1).WithMagnitude(1);
    Vector isoV = Vector::From(1, 1, 1).WithMagnitude(1).Cross(isoU).WithMagnitude(1);
    ViewDef allViews[kNumDrawingViews] = {
        { DrawingViewName(0), Vector::From( 1, 0,  0), Vector::From(0, 1,  0) }, // front
        { DrawingViewName(1), Vector::From(-1, 0,  0), Vector::From(0, 1,  0) }, // back
        { DrawingViewName(2), Vector::From( 0, 0,  1), Vector::From(0, 1,  0) }, // left
        { DrawingViewName(3), Vector::From( 0, 0, -1), Vector::From(0, 1,  0) }, // right
        { DrawingViewName(4), Vector::From( 1, 0,  0), Vector::From(0, 0, -1) }, // top
        { DrawingViewName(5), Vector::From( 1, 0,  0), Vector::From(0, 0,  1) }, // bottom
        { DrawingViewName(6), isoU, isoV },                                      // iso
        { DrawingViewName(7), savedRight, savedUp },                             // current
    };
    std::vector<ViewDef> views;
    for(int i = 0; i < kNumDrawingViews; i++) {
        if(SS.drawingViewMask & (1u << i)) views.push_back(allViews[i]);
    }
    if(views.empty()) views.push_back(allViews[0]);
    SS.exportScale  = 1.0;
    SS.exportOffset = 0.0;

    std::vector<CapturedView> captured(views.size());
    Group *g = SK.GetGroup(SS.GW.activeGroup);

    // Dimension text is drawn at Style::TextHeight (in model units for MM
    // styles). Override the constraint style so the text is a fixed fraction
    // of the part; since the layout scale below is ~cell/part-size, this lands
    // dimensions at a roughly constant, legible size on the sheet regardless of
    // part size or current zoom. (A proper sheet-relative size belongs with the
    // page/scale settings in phase 2.)
    Style *cstyle = Style::Get({ Style::CONSTRAINT });
    double          savedTextHeight   = cstyle->textHeight;
    Style::UnitsAs  savedTextHeightAs = cstyle->textHeightAs;
    g->GenerateDisplayItems();
    if(!g->displayMesh.IsEmpty()) {
        Vector bmax, bmin;
        g->displayMesh.GetBounding(&bmax, &bmin);
        Vector d = bmax.Minus(bmin);
        double extent = max(d.x, max(d.y, d.z));
        if(extent > LENGTH_EPS) {
            // ~label size on a typical sheet; tune if dims read large/small.
            cstyle->textHeight   = 0.022 * extent;
            cstyle->textHeightAs = Style::UnitsAs::MM;
            // Pin the view scale too, so dimension arrows/gaps (sized in
            // pixels = 1/scale) don't track the user's zoom. 250/extent matches
            // the familiar default look (default scale 5 at extent 50).
            SS.GW.scale = 250.0 / extent;
        }
    }

    for(size_t i = 0; i < views.size(); i++) {
        Vector u = views[i].u, v = views[i].v, n = u.Cross(v);
        SS.GW.projRight = u;
        SS.GW.projUp    = v;
        g->GenerateDisplayItems();

        SMesh *sm = &(g->displayMesh);
        if(sm->IsEmpty()) sm = NULL;

        SEdgeList edges = {};
        SBezierList beziers = {};
        for(auto &entity : SK.entity) {
            Entity *e = &entity;
            if(!e->IsVisible()) continue;
            e->GenerateEdges(&edges);
        }
        if(SS.GW.showEdges) {
            g->displayOutlines.ListTaggedInto(&edges, Style::SOLID_EDGE);
        }
        // Reference dimensions / constraints, projected into this view.
        if(SS.GW.showConstraints != GraphicsWindow::ShowConstraintMode::SCM_NOSHOW) {
            GetEdgesCanvas canvas = {};
            canvas.camera = SS.GW.GetCamera();
            canvas.edges  = &edges;
            for(Constraint &c : SK.constraint) {
                c.Draw(Constraint::DrawAs::DEFAULT, &canvas);
            }
            canvas.Clear();
        }

        // Orthographic projection (cameraTan = 0); occlusion against the mesh
        // gives proper hidden-line treatment.
        Vector origin = Vector::From(0, 0, 0);
        captured[i].SetModelviewProjection(u, v, n, origin, 0.0, SS.exportScale);
        ExportLinesAndMesh(&edges, &beziers, sm, u, v, n, origin, 0.0, &captured[i]);

        edges.Clear();
        beziers.Clear();
    }

    // Restore view state and the constraint style.
    SS.GW.projRight = savedRight;
    SS.GW.projUp    = savedUp;
    SS.exportScale  = savedScale;
    SS.exportOffset = savedOffset;
    SS.GW.scale     = savedGwScale;
    cstyle->textHeight   = savedTextHeight;
    cstyle->textHeightAs = savedTextHeightAs;
    g->GenerateDisplayItems();

    // Bounding box of each view, and the largest extents (for a common scale).
    struct Box { Vector min, max; };
    std::vector<Box> boxes(views.size());
    double maxW = 0, maxH = 0;
    for(size_t i = 0; i < views.size(); i++) {
        Vector mn = Vector::From(VERY_POSITIVE, VERY_POSITIVE, 0);
        Vector mx = Vector::From(VERY_NEGATIVE, VERY_NEGATIVE, 0);
        bool any = false;
        for(auto &st : captured[i].strokes) {
            for(auto &sb : st.beziers) {
                for(int k = 0; k <= sb.deg; k++) { sb.ctrl[k].MakeMaxMin(&mx, &mn); any = true; }
            }
        }
        if(!any) { mn = Vector::From(0, 0, 0); mx = Vector::From(0, 0, 0); }
        boxes[i].min = mn;
        boxes[i].max = mx;
        maxW = max(maxW, mx.x - mn.x);
        maxH = max(maxH, mx.y - mn.y);
    }
    if(maxW < LENGTH_EPS) maxW = 1;
    if(maxH < LENGTH_EPS) maxH = 1;

    double pageW, pageH;
    DrawingPaperSize(SS.drawingPaperSize, NULL, &pageW, &pageH);
    if(SS.drawingLandscape) swap(pageW, pageH);
    double margin = max(0.0, SS.drawingMargin);
    double usableW = pageW - 2 * margin, usableH = pageH - 2 * margin;

    // Reserve a bottom strip for the title block; the views lay out above it.
    double titleH = SS.drawingTitleBlock ? min(22.0, usableH * 0.5) : 0.0;
    double viewsAreaH = usableH - titleH;

    // Lay the views out as a grid, optionally split across pages: 0
    // views-per-page means all on one sheet; otherwise that many cells per page
    // (only when the caller permits, i.e. PDF; other formats are single-page).
    // A single common (to-scale) factor fits the largest view into a cell.
    int nViews = (int)views.size();
    bool multiPage = allowMultiPage && SS.drawingViewsPerPage >= 1;
    int cellsPerPage = (multiPage && SS.drawingViewsPerPage < nViews)
                           ? SS.drawingViewsPerPage : nViews;
    if(cellsPerPage < 1) cellsPerPage = 1;
    double gap = margin * 0.8;
    int cols = (int)ceil(sqrt((double)cellsPerPage));
    if(cols < 1) cols = 1;
    int rows = (cellsPerPage + cols - 1) / cols;
    double cellW = max(1.0, usableW / cols - gap);
    double cellH = max(1.0, viewsAreaH / rows - gap);
    double scale = min(cellW / maxW, cellH / maxH);

    sheet->pageW  = pageW;
    sheet->pageH  = pageH;
    sheet->nPages = (nViews + cellsPerPage - 1) / cellsPerPage;
    sheet->items.resize(nViews);

    for(int i = 0; i < nViews; i++) {
        int idx = i % cellsPerPage;
        int col = idx % cols, row = idx / cols;
        double rowBottom = pageH - margin - (row + 1) * (viewsAreaH / rows);
        double cellX0 = margin + col * (usableW / cols) + gap / 2;
        double cellY0 = rowBottom + gap / 2;

        double w = (boxes[i].max.x - boxes[i].min.x) * scale;
        double h = (boxes[i].max.y - boxes[i].min.y) * scale;
        double ox = cellX0 + (cellW - w) / 2 - boxes[i].min.x * scale;
        double oy = cellY0 + (cellH - h) / 2 - boxes[i].min.y * scale;

        DrawingSheet::Item &item = sheet->items[i];
        item.page = i / cellsPerPage;

        // View linework, placed in page mm.
        for(auto &st : captured[i].strokes) {
            DrawingSheet::Stroke ds;
            // Use the chosen sheet line weight rather than the captured width,
            // which is pixels/zoom (Style::WidthMm) and thus depends on the
            // current zoom level.
            ds.strokeRgb = st.strokeRgb; ds.lineWidth = SS.drawingLineWidth;
            ds.filled    = st.filled;    ds.fillRgb   = st.fillRgb;  ds.hs = st.hs;
            for(auto &sb : st.beziers) {
                SBezier t = sb;
                for(int k = 0; k <= t.deg; k++) {
                    t.ctrl[k] = Vector::From(sb.ctrl[k].x * scale + ox,
                                             sb.ctrl[k].y * scale + oy, 0);
                }
                ds.beziers.push_back(t);
            }
            item.strokes.push_back(ds);
        }

        // View name label, drawn as linework so it renders the same way
        // wherever the sheet is consumed.
        double lh = 2.8;
        std::string label = views[i].name;
        double lw = VectorFont::Builtin()->GetWidth(lh, label);
        double labelX = margin + col * (usableW / cols) + (usableW / cols - lw) / 2;
        double labelY = rowBottom + 0.6;
        DrawingSheet::Stroke lab;
        lab.strokeRgb = RGBi(0, 0, 0); lab.lineWidth = 0.25;
        lab.filled    = false;         lab.fillRgb   = RGBi(0, 0, 0);
        lab.hs        = { Style::CONSTRAINT };
        VectorFont::Builtin()->Trace(lh, Vector::From(labelX, labelY, 0),
            Vector::From(1, 0, 0), Vector::From(0, 1, 0), label,
            [&](Vector a, Vector b) { lab.beziers.push_back(SBezier::From(a, b)); });
        item.strokes.push_back(lab);
    }

    // Title block, one per page (each carrying that page's sheet number).
    if(SS.drawingTitleBlock && titleH > LENGTH_EPS) {
        std::string name = SS.saveFile.FileStem();
        if(name.empty()) name = "untitled";
        std::string scaleStr = (scale >= 1.0) ? ssprintf("%.3g:1", scale)
                                              : ssprintf("1:%.3g", 1.0 / scale);
        char datebuf[32] = "";
        time_t t = time(NULL);
        struct tm *lt = localtime(&t);
        if(lt) strftime(datebuf, sizeof(datebuf), "%Y-%m-%d", lt);

        double bh = max(1.0, titleH - gap);
        double bw = min(usableW, 150.0);
        double bl = pageW - margin - bw, bb = margin;
        for(int pg = 0; pg < sheet->nPages; pg++) {
            DrawingSheet::Item blk;
            blk.page = pg;
            AddTitleBlock(&blk, bl, bb, bw, bh, SS.drawingTitle, name, scaleStr,
                          SS.UnitName(), datebuf, SS.drawingDrawnBy, SS.drawingRev,
                          pg, sheet->nPages);
            // Fill the leftover space left of the block with a notes box.
            double nW = (bl - gap) - margin;
            if(SS.drawingNotesBox && nW > 25.0) {
                AddNotesBox(&blk, margin, bb, nW, bh);
            }
            sheet->items.push_back(blk);
        }
    }
}

void SolveSpaceUI::ExportDrawingViewsTo(const Platform::Path &filename) {
    SS.exportMode = true;
    GenerateAll(Generate::ALL);

    DrawingSheet sheet;
    ComputeDrawingSheet(&sheet, /*allowMultiPage=*/filename.HasExtension("pdf"));

    VectorFileWriter *out = VectorFileWriter::ForFile(filename);
    if(!out) return;
    out->ptMin = Vector::From(0, 0, 0);
    out->ptMax = Vector::From(sheet.pageW, sheet.pageH, 0);
    out->StartFile();
    for(int pg = 0; pg < sheet.nPages; pg++) {
        if(pg > 0) out->NewPage();
        for(auto &item : sheet.items) {
            if(item.page != pg) continue;
            for(auto &st : item.strokes) {
                out->StartPath(st.strokeRgb, st.lineWidth, st.filled, st.fillRgb, st.hs);
                for(auto &b : st.beziers) out->Bezier(&b);
                out->FinishPath(st.strokeRgb, st.lineWidth, st.filled, st.fillRgb, st.hs);
            }
        }
    }
    out->FinishAndCloseFile();
}

void SolveSpaceUI::ExportLinesAndMesh(SEdgeList *sel, SBezierList *sbl, SMesh *sm,
                                      Vector u, Vector v, Vector n,
                                      Vector origin, double cameraTan,
                                      VectorFileWriter *out)
{
    double s = 1.0 / SS.exportScale;

    // Project into the export plane; so when we're done, z doesn't matter,
    // and x and y are what goes in the DXF.
    for(SEdge *e = sel->l.First(); e; e = sel->l.NextAfter(e)) {
        // project into the specified csys, and apply export scale
        (e->a) = e->a.InPerspective(u, v, n, origin, cameraTan).ScaledBy(s);
        (e->b) = e->b.InPerspective(u, v, n, origin, cameraTan).ScaledBy(s);
    }

    if(sbl) {
        for(SBezier *b = sbl->l.First(); b; b = sbl->l.NextAfter(b)) {
            *b = b->InPerspective(u, v, n, origin, cameraTan);
            int i;
            for(i = 0; i <= b->deg; i++) {
                b->ctrl[i] = (b->ctrl[i]).ScaledBy(s);
            }
        }
    }

    // If cutter radius compensation is requested, then perform it now
    if(fabs(SS.exportOffset) > LENGTH_EPS) {
        // assemble those edges into a polygon, and clear the edge list
        SPolygon sp = {};
        sel->AssemblePolygon(&sp, NULL);
        sel->Clear();

        SPolygon compd = {};
        sp.normal = {0, 0, -1};
        sp.FixContourDirections();
        sp.OffsetInto(&compd, SS.exportOffset*s);
        sp.Clear();

        compd.MakeEdgesInto(sel);
        compd.Clear();
    }

    // Now the triangle mesh; project, then build a BSP to perform
    // occlusion testing and generated the shaded surfaces.
    SMesh smp = {};
    if(sm) {
        Vector l0 = (SS.lightDir[0]).WithMagnitude(1),
               l1 = (SS.lightDir[1]).WithMagnitude(1);
        STriangle *tr;
        for(tr = sm->l.First(); tr; tr = sm->l.NextAfter(tr)) {
            STriangle tt = *tr;
            tt.a = (tt.a).InPerspective(u, v, n, origin, cameraTan).ScaledBy(s);
            tt.b = (tt.b).InPerspective(u, v, n, origin, cameraTan).ScaledBy(s);
            tt.c = (tt.c).InPerspective(u, v, n, origin, cameraTan).ScaledBy(s);

            // And calculate lighting for the triangle
            Vector n = tt.Normal().WithMagnitude(1);
            double lighting = min(1.0, SS.ambientIntensity +
                                  max(0.0, (SS.lightIntensity[0])*(n.Dot(l0))) +
                                  max(0.0, (SS.lightIntensity[1])*(n.Dot(l1))));
            double r = min(1.0, tt.meta.color.redF()   * lighting),
                   g = min(1.0, tt.meta.color.greenF() * lighting),
                   b = min(1.0, tt.meta.color.blueF()  * lighting);
            tt.meta.color = RGBf(r, g, b);
            smp.AddTriangle(&tt);
        }
    }

    SMesh sms = {};

    // We need the mesh for occlusion testing, but if we don't/can't export it,
    // don't generate it.
    if(SS.GW.showShaded && out->CanOutputMesh()) {
        // Use the BSP routines to generate the split triangles in paint order.
        SBsp3 *bsp = SBsp3::FromMesh(&smp);
        if(bsp) bsp->GenerateInPaintOrder(&sms);
        // And cull the back-facing triangles
        STriangle *tr;
        sms.l.ClearTags();
        for(tr = sms.l.First(); tr; tr = sms.l.NextAfter(tr)) {
            Vector n = tr->Normal();
            if(n.z < 0) {
                tr->tag = 1;
            }
        }
        sms.l.RemoveTagged();
    }

    // And now we perform hidden line removal if requested
    SEdgeList hlrd = {};
    if(sm) {
        SKdNode *root = SKdNode::From(&smp);

        // Generate the edges where a curved surface turns from front-facing
        // to back-facing.
        if(SS.GW.showEdges || SS.GW.showOutlines) {
            root->MakeCertainEdgesInto(sel, EdgeKind::TURNING,
                                       /*coplanarIsInter=*/false, NULL, NULL,
                                       GW.showOutlines ? Style::OUTLINE : Style::SOLID_EDGE);
        }

        root->ClearTags();
        int cnt = 1234;

        SEdge *se;
        for(se = sel->l.First(); se; se = sel->l.NextAfter(se)) {
            if(se->auxA == Style::CONSTRAINT) {
                // Constraints should not get hidden line removed; they're
                // always on top.
                hlrd.AddEdge(se->a, se->b, se->auxA);
                continue;
            }

            SEdgeList edges = {};
            // Split the original edge against the mesh
            edges.AddEdge(se->a, se->b, se->auxA);
            root->OcclusionTestLine(*se, &edges, cnt);
            if(SS.GW.drawOccludedAs == GraphicsWindow::DrawOccludedAs::STIPPLED) {
                for(SEdge &se : edges.l) {
                    if(se.tag == 1) {
                        se.auxA = Style::HIDDEN_EDGE;
                    }
                }
            } else if(SS.GW.drawOccludedAs == GraphicsWindow::DrawOccludedAs::INVISIBLE) {
                edges.l.RemoveTagged();
            }

            // the occlusion test splits unnecessarily; so fix those
            edges.MergeCollinearSegments(se->a, se->b);
            cnt++;
            // And add the results to our output
            SEdge *sen;
            for(sen = edges.l.First(); sen; sen = edges.l.NextAfter(sen)) {
                hlrd.AddEdge(sen->a, sen->b, sen->auxA);
            }
            edges.Clear();
        }

        sel = &hlrd;
    }

    // Clean up: remove overlapping line segments and
    // segments with zero-length projections.
    sel->l.ClearTags();
    for(int i = 0; i < sel->l.n; ++i) {
        SEdge *sei = &sel->l[i];
        hStyle hsi = { (uint32_t)sei->auxA };
        Style *si = Style::Get(hsi);
        if(sei->tag != 0) continue;

        // Remove segments with zero length projections.
        Vector ai = sei->a;
        ai.z = 0.0;
        Vector bi = sei->b;
        bi.z = 0.0;
        Vector di = bi.Minus(ai);
        if(fabs(di.x) < LENGTH_EPS && fabs(di.y) < LENGTH_EPS) {
            sei->tag = 1;
            continue;
        }

        for(int j = i + 1; j < sel->l.n; ++j) {
            SEdge *sej = &sel->l[j];
            if(sej->tag != 0) continue;

            Vector *pAj = &sej->a;
            Vector *pBj = &sej->b;

            // Remove segments with zero length projections.
            Vector aj = sej->a;
            aj.z = 0.0;
            Vector bj = sej->b;
            bj.z = 0.0;
            Vector dj = bj.Minus(aj);
            if(fabs(dj.x) < LENGTH_EPS && fabs(dj.y) < LENGTH_EPS) {
                sej->tag = 1;
                continue;
            }

            // Skip non-collinear segments.
            const double eps = 1e-6;
            if(aj.DistanceToLine(ai, di) > eps) continue;
            if(bj.DistanceToLine(ai, di) > eps) continue;

            double ta = aj.Minus(ai).Dot(di) / di.Dot(di);
            double tb = bj.Minus(ai).Dot(di) / di.Dot(di);
            if(ta > tb) {
                std::swap(pAj, pBj);
                std::swap(ta, tb);
            }

            hStyle hsj = { (uint32_t)sej->auxA };
            Style *sj = Style::Get(hsj);

            bool canRemoveI = sej->auxA == sei->auxA || si->zIndex < sj->zIndex;
            bool canRemoveJ = sej->auxA == sei->auxA || sj->zIndex < si->zIndex;

            if(canRemoveJ) {
                // j-segment inside i-segment
                if(ta > 0.0 - eps && tb < 1.0 + eps) {
                    sej->tag = 1;
                    continue;
                }

                // cut segment
                bool aInside = ta > 0.0 - eps && ta < 1.0 + eps;
                if(tb > 1.0 - eps && aInside) {
                    *pAj = sei->b;
                    continue;
                }

                // cut segment
                bool bInside = tb > 0.0 - eps && tb < 1.0 + eps;
                if(ta < 0.0 - eps && bInside) {
                    *pBj = sei->a;
                    continue;
                }

                // split segment
                if(ta < 0.0 - eps && tb > 1.0 + eps) {
                    sel->AddEdge(sei->b, *pBj, sej->auxA, sej->auxB);
                    *pBj = sei->a;
                    continue;
                }
            }

            if(canRemoveI) {
                // j-segment inside i-segment
                if(ta < 0.0 + eps && tb > 1.0 - eps) {
                    sei->tag = 1;
                    break;
                }

                // cut segment
                bool aInside = ta > 0.0 + eps && ta < 1.0 - eps;
                if(tb > 1.0 - eps && aInside) {
                    sei->b = *pAj;
                    i--;
                    break;
                }

                // cut segment
                bool bInside = tb > 0.0 + eps && tb < 1.0 - eps;
                if(ta < 0.0 + eps && bInside) {
                    sei->a = *pBj;
                    i--;
                    break;
                }

                // split segment
                if(ta > 0.0 + eps && tb < 1.0 - eps) {
                    sel->AddEdge(*pBj, sei->b, sei->auxA, sei->auxB);
                    sei->b = *pAj;
                    i--;
                    break;
                }
            }
        }
    }
    sel->l.RemoveTagged();

    // We kept the line segments and Beziers separate until now; but put them
    // all together, and also project everything into the xy plane, since not
    // all export targets ignore the z component of the points.
    ssassert(sbl != nullptr, "Adding line segments to beziers assumes bezier list is non-null.");
    for(SEdge *e = sel->l.First(); e; e = sel->l.NextAfter(e)) {
        SBezier sb = SBezier::From(e->a, e->b);
        sb.auxA = e->auxA;
        sbl->l.Add(&sb);
    }
    for(SBezier *b = sbl->l.First(); b; b = sbl->l.NextAfter(b)) {
        for(int i = 0; i <= b->deg; i++) {
            b->ctrl[i].z = 0;
        }
    }

    // If possible, then we will assemble these output curves into loops. They
    // will then get exported as closed paths.
    SBezierLoopSetSet sblss = {};
    SBezierLoopSet leftovers = {};
    SSurface srf = SSurface::FromPlane({0, 0, 0},
                                       {1, 0, 0},
                                       {0, 1, 0});
    SPolygon spxyz = {};
    bool allClosed;
    SEdge notClosedAt;
    sbl->l.ClearTags();
    sblss.FindOuterFacesFrom(sbl, &spxyz, &srf,
                             SS.ExportChordTolMm(),
                             &allClosed, &notClosedAt,
                             NULL, NULL,
                             &leftovers);
    sblss.l.Add(&leftovers);

    // Now write the lines and triangles to the output file
    out->OutputLinesAndMesh(&sblss, &sms);

    spxyz.Clear();
    sblss.Clear();
    smp.Clear();
    sms.Clear();
    hlrd.Clear();
}

double VectorFileWriter::MmToPts(double mm) {
    // 72 points in an inch
    return (mm/25.4)*72;
}

VectorFileWriter *VectorFileWriter::ForFile(const Platform::Path &filename) {
    VectorFileWriter *ret;
    bool needOpen = true;
    if(filename.HasExtension("dxf")) {
        static DxfFileWriter DxfWriter;
        ret = &DxfWriter;
        needOpen = false;
    } else if(filename.HasExtension("ps") || filename.HasExtension("eps")) {
        static EpsFileWriter EpsWriter;
        ret = &EpsWriter;
    } else if(filename.HasExtension("pdf")) {
        static PdfFileWriter PdfWriter;
        ret = &PdfWriter;
    } else if(filename.HasExtension("svg")) {
        static SvgFileWriter SvgWriter;
        ret = &SvgWriter;
    } else if(filename.HasExtension("plt") || filename.HasExtension("hpgl")) {
        static HpglFileWriter HpglWriter;
        ret = &HpglWriter;
    } else if(filename.HasExtension("step") || filename.HasExtension("stp")) {
        static Step2dFileWriter Step2dWriter;
        ret = &Step2dWriter;
    } else if(filename.HasExtension("txt") || filename.HasExtension("ngc")) {
        static GCodeFileWriter GCodeWriter;
        ret = &GCodeWriter;
    } else {
        Error("Can't identify output file type from file extension of "
        "filename '%s'; try "
        ".step, .stp, .dxf, .svg, .plt, .hpgl, .pdf, .txt, .ngc, "
        ".eps, or .ps.",
            filename.raw.c_str());
        return NULL;
    }
    ret->filename = filename;
    if(!needOpen) return ret;

    FILE *f = OpenFile(filename, "wb");
    if(!f) {
        Error("Couldn't write to '%s'", filename.raw.c_str());
        return NULL;
    }
    ret->f = f;
    return ret;
}

void VectorFileWriter::SetModelviewProjection(const Vector &u, const Vector &v, const Vector &n,
                                              const Vector &origin, double cameraTan,
                                              double scale) {
    this->u = u;
    this->v = v;
    this->n = n;
    this->origin = origin;
    this->cameraTan = cameraTan;
    this->scale = scale;
}

Vector VectorFileWriter::Transform(Vector &pos) const {
    return pos.InPerspective(u, v, n, origin, cameraTan).ScaledBy(1.0 / scale);
}

void VectorFileWriter::OutputLinesAndMesh(SBezierLoopSetSet *sblss, SMesh *sm) {
    STriangle *tr;
    SBezier *b;

    // First calculate the bounding box.
    ptMin = {VERY_POSITIVE, VERY_POSITIVE, VERY_POSITIVE};
    ptMax = {VERY_NEGATIVE, VERY_NEGATIVE, VERY_NEGATIVE};
    if(sm) {
        for(tr = sm->l.First(); tr; tr = sm->l.NextAfter(tr)) {
            (tr->a).MakeMaxMin(&ptMax, &ptMin);
            (tr->b).MakeMaxMin(&ptMax, &ptMin);
            (tr->c).MakeMaxMin(&ptMax, &ptMin);
        }
    }
    if(sblss) {
        SBezierLoopSet *sbls;
        for(sbls = sblss->l.First(); sbls; sbls = sblss->l.NextAfter(sbls)) {
            SBezierLoop *sbl;
            for(sbl = sbls->l.First(); sbl; sbl = sbls->l.NextAfter(sbl)) {
                for(b = sbl->l.First(); b; b = sbl->l.NextAfter(b)) {
                    for(int i = 0; i <= b->deg; i++) {
                        (b->ctrl[i]).MakeMaxMin(&ptMax, &ptMin);
                    }
                }
            }
        }
    }

    // And now we compute the canvas size.
    double s = 1.0 / SS.exportScale;
    if(SS.exportCanvasSizeAuto) {
        // It's based on the calculated bounding box; we grow it along each
        // boundary by the specified amount.
        ptMin.x -= s*SS.exportMargin.left;
        ptMax.x += s*SS.exportMargin.right;
        ptMin.y -= s*SS.exportMargin.bottom;
        ptMax.y += s*SS.exportMargin.top;
    } else {
        ptMin.x = (s*SS.exportCanvas.dx);
        ptMin.y = (s*SS.exportCanvas.dy);
        ptMax.x = ptMin.x + (s*SS.exportCanvas.width);
        ptMax.y = ptMin.y + (s*SS.exportCanvas.height);
    }

    StartFile();
    if(SS.exportBackgroundColor) {
        Background(SS.backgroundColor);
    }
    if(sm && SS.exportShadedTriangles) {
        for(tr = sm->l.First(); tr; tr = sm->l.NextAfter(tr)) {
            Triangle(tr);
        }
    }
    if(sblss) {
        SBezierLoopSet *sbls;
        for(sbls = sblss->l.First(); sbls; sbls = sblss->l.NextAfter(sbls)) {
            for(SBezierLoop *sbl = sbls->l.First(); sbl; sbl = sbls->l.NextAfter(sbl)) {
                b = sbl->l.First();
                if(!b || !Style::Exportable(b->auxA)) continue;

                hStyle hs = { (uint32_t)b->auxA };
                Style *stl = Style::Get(hs);
                double lineWidth   = Style::WidthMm(b->auxA)*s;
                RgbaColor strokeRgb = Style::Color(hs, /*forExport=*/true);
                RgbaColor fillRgb   = Style::FillColor(hs, /*forExport=*/true);

                StartPath(strokeRgb, lineWidth, stl->filled, fillRgb, hs);
                for(b = sbl->l.First(); b; b = sbl->l.NextAfter(b)) {
                    Bezier(b);
                }
                FinishPath(strokeRgb, lineWidth, stl->filled, fillRgb, hs);
            }
        }
    }
    FinishAndCloseFile();
}

void VectorFileWriter::BezierAsPwl(SBezier *sb) {
    List<Vector> lv = {};
    sb->MakePwlInto(&lv, SS.ExportChordTolMm());

    for(int i = 1; i < lv.n; i++) {
        SBezier sb = SBezier::From(lv[i-1], lv[i]);
        Bezier(&sb);
    }
    lv.Clear();
}

void VectorFileWriter::BezierAsNonrationalCubic(SBezier *sb, int depth) {
    Vector t0 = sb->TangentAt(0), t1 = sb->TangentAt(1);
    // The curve is correct, and the first derivatives are correct, at the
    // endpoints.
    SBezier bnr = SBezier::From(
                        sb->Start(),
                        sb->Start().Plus(t0.ScaledBy(1.0/3)),
                        sb->Finish().Minus(t1.ScaledBy(1.0/3)),
                        sb->Finish());

    double tol = SS.ExportChordTolMm();
    // Arbitrary choice, but make it a little finer than pwl tolerance since
    // it should be easier to achieve that with the smooth curves.
    tol /= 2;

    bool closeEnough = true;
    int i;
    for(i = 1; i <= 3; i++) {
        double t = i/4.0;
        Vector p0 = sb->PointAt(t),
               pn = bnr.PointAt(t);
        double d = (p0.Minus(pn)).Magnitude();
        if(d > tol) {
            closeEnough = false;
        }
    }

    if(closeEnough || depth > 3) {
        Bezier(&bnr);
    } else {
        SBezier bef, aft;
        sb->SplitAt(0.5, &bef, &aft);
        BezierAsNonrationalCubic(&bef, depth+1);
        BezierAsNonrationalCubic(&aft, depth+1);
    }
}

//-----------------------------------------------------------------------------
// Export a triangle mesh, in the requested format.
//-----------------------------------------------------------------------------
void SolveSpaceUI::ExportMeshTo(const Platform::Path &filename) {
    SS.exportMode = true;
    GenerateAll(Generate::ALL);

    Group *g = SK.GetGroup(SS.GW.activeGroup);
    g->GenerateDisplayItems();

    SMesh *m = &(SK.GetGroup(SS.GW.activeGroup)->displayMesh);
    if(m->IsEmpty()) {
        Error(_("Active group mesh is empty; nothing to export."));
        return;
    }

    FILE *f = OpenFile(filename, "wb");
    if(!f) {
        Error("Couldn't write to '%s'", filename.raw.c_str());
        return;
    }
    ShowNakedEdges(/*reportOnlyWhenNotOkay=*/true);
    if(filename.HasExtension("stl")) {
        ExportMeshAsStlTo(f, m);
    } else if(filename.HasExtension("obj")) {
        Platform::Path mtlFilename = filename.WithExtension("mtl");
        FILE *fMtl = OpenFile(mtlFilename, "wb");
        if(!fMtl) {
            Error("Couldn't write to '%s'", filename.raw.c_str());
            return;
        }

        fprintf(f, "mtllib %s\n", mtlFilename.FileName().c_str());
        ExportMeshAsObjTo(f, fMtl, m);

        fclose(fMtl);
    } else if(filename.HasExtension("js") ||
              filename.HasExtension("html")) {
        SOutlineList *e = &(SK.GetGroup(SS.GW.activeGroup)->displayOutlines);
        ExportMeshAsThreeJsTo(f, filename, m, e);
    } else if(filename.HasExtension("wrl")) {
        ExportMeshAsVrmlTo(f, filename, m);
    } else {
        Error("Can't identify output file type from file extension of "
              "filename '%s'; try .stl, .obj, .js, .html.", filename.raw.c_str());
    }

    fclose(f);

    SS.justExportedInfo.showOrigin = false;
    SS.justExportedInfo.draw = true;
    GW.Invalidate();
}

//-----------------------------------------------------------------------------
// Export the mesh as an STL file; it should always be vertex-to-vertex and
// not self-intersecting, so not much to do.
//-----------------------------------------------------------------------------
void SolveSpaceUI::ExportMeshAsStlTo(FILE *f, SMesh *sm) {
    char str[80] = {};
    strcpy(str, "STL exported mesh");
    fwrite(str, 1, 80, f);

    uint32_t n = sm->l.n;
    fwrite(&n, 4, 1, f);

    double s = SS.exportScale;
    int i;
    for(i = 0; i < sm->l.n; i++) {
        STriangle *tr = &(sm->l[i]);
        Vector n = tr->Normal().WithMagnitude(1);
        float w;
        w = (float)n.x;           fwrite(&w, 4, 1, f);
        w = (float)n.y;           fwrite(&w, 4, 1, f);
        w = (float)n.z;           fwrite(&w, 4, 1, f);
        w = (float)((tr->a.x)/s); fwrite(&w, 4, 1, f);
        w = (float)((tr->a.y)/s); fwrite(&w, 4, 1, f);
        w = (float)((tr->a.z)/s); fwrite(&w, 4, 1, f);
        w = (float)((tr->b.x)/s); fwrite(&w, 4, 1, f);
        w = (float)((tr->b.y)/s); fwrite(&w, 4, 1, f);
        w = (float)((tr->b.z)/s); fwrite(&w, 4, 1, f);
        w = (float)((tr->c.x)/s); fwrite(&w, 4, 1, f);
        w = (float)((tr->c.y)/s); fwrite(&w, 4, 1, f);
        w = (float)((tr->c.z)/s); fwrite(&w, 4, 1, f);
        fputc(0, f);
        fputc(0, f);
    }
}

//-----------------------------------------------------------------------------
// Export the mesh as Wavefront OBJ format. This requires us to reduce all the
// identical vertices to the same identifier, so do that first.
//-----------------------------------------------------------------------------
void SolveSpaceUI::ExportMeshAsObjTo(FILE *fObj, FILE *fMtl, SMesh *sm) {
    std::map<RgbaColor, std::string, RgbaColorCompare> colors;
    for(const STriangle &t : sm->l) {
        RgbaColor color = t.meta.color;
        if(colors.find(color) == colors.end()) {
            std::string id = ssprintf("h%02x%02x%02x",
                                      color.red,
                                      color.green,
                                      color.blue);
            colors.emplace(color, id);
        }
        for(int i = 0; i < 3; i++) {
            fprintf(fObj, "v %.10f %.10f %.10f\n",
                    CO(t.vertices[i].ScaledBy(1 / SS.exportScale)));
        }
    }

    for(auto &it : colors) {
        fprintf(fMtl, "newmtl %s\n",
                it.second.c_str());
        fprintf(fMtl, "Kd %.3f %.3f %.3f\n",
                it.first.redF(), it.first.greenF(), it.first.blueF());
    }

    for(const STriangle &t : sm->l) {
        for(int i = 0; i < 3; i++) {
            Vector n = t.normals[i].WithMagnitude(1.0);
            fprintf(fObj, "vn %.10f %.10f %.10f\n",
                    CO(n));
        }
    }

    RgbaColor currentColor = {};
    for(int i = 0; i < sm->l.n; i++) {
        const STriangle &t = sm->l[i];
        if(!currentColor.Equals(t.meta.color)) {
            currentColor = t.meta.color;
            fprintf(fObj, "usemtl %s\n", colors[currentColor].c_str());
        }

        fprintf(fObj, "f %d//%d %d//%d %d//%d\n",
                i * 3 + 1, i * 3 + 1,
                i * 3 + 2, i * 3 + 2,
                i * 3 + 3, i * 3 + 3);
    }
}

//-----------------------------------------------------------------------------
// Export the mesh as a JavaScript script, which is compatible with Three.js.
//-----------------------------------------------------------------------------
void SolveSpaceUI::ExportMeshAsThreeJsTo(FILE *f, const Platform::Path &filename,
                                         SMesh *sm, SOutlineList *sol)
{
    SPointList spl = {};
    STriangle *tr;
    Vector bndl, bndh;

    const std::string THREE_FN("three-r111.min.js");
    const std::string HAMMER_FN("hammer-2.0.8.js");
    const std::string CONTROLS_FN("SolveSpaceControls.js");

    const char htmlbegin[] = R"(
<!DOCTYPE html>
<html lang="en">
  <head>
    <meta charset="utf-8"></meta>
    <title>Three.js Solvespace Mesh</title>
    <script id="%s">%s</script>
    <script id="%s">%s</script>
    <script id="%s">%s</script>
    <style type="text/css">
    body { margin: 0; overflow: hidden; }
    </style>
  </head>
  <body>
    <script>
)";
    const char htmlend[] = R"(
    document.body.appendChild(solvespace(solvespace_model_%s, {
        scale: %g,
        offset: new THREE.Vector3(%g, %g, %g),
        projUp: new THREE.Vector3(%g, %g, %g),
        projRight: new THREE.Vector3(%g, %g, %g)
    }));
    </script>
  </body>
</html>
)";

    // A default three.js viewer with OrthographicTrackballControls is
    // generated as a comment preceding the data.

    // x bounds should be the range of x or y, whichever
    // is larger, before aspect ratio correction is applied.
    // y bounds should be the range of x or y, whichever is
    // larger. No aspect ratio correction is applied.
    // Near plane should be 1.
    // Camera's z-position should be the range of z + 1 or the larger of
    // the x or y bounds, whichever is larger.
    // Far plane should be at least twice as much as the camera's
    // z-position.
    // Edge projection bias should be about 1/500 of the far plane's distance.
    // Further corrections will be applied to the z-position and far plane in
    // the default viewer, but the defaults are fine for a model which
    // only rotates about the world origin.

    sm->GetBounding(&bndh, &bndl);
    double largerBoundXY = max((bndh.x - bndl.x), (bndh.y - bndl.y));
    double largerBoundZ = max(largerBoundXY, (bndh.z - bndl.z + 1));

    std::string basename = filename.FileStem();
    for(size_t i = 0; i < basename.length(); i++) {
        if(!(isalnum(basename[i]) || ((unsigned)basename[i] >= 0x80))) {
            basename[i] = '_';
        }
    }

    if(filename.HasExtension("html")) {
        fprintf(f, htmlbegin,
                THREE_FN.c_str(),
                LoadStringFromGzip("threejs/" + THREE_FN + ".gz").c_str(),
                HAMMER_FN.c_str(),
                LoadStringFromGzip("threejs/" + HAMMER_FN + ".gz").c_str(),
                CONTROLS_FN.c_str(),
                LoadString("threejs/" + CONTROLS_FN).c_str());
    }

    fprintf(f, "var solvespace_model_%s = {\n"
               "  bounds: {\n"
               "    x: %f, y: %f, near: %f, far: %f, z: %f, edgeBias: %f\n"
               "  },\n",
            basename.c_str(),
            largerBoundXY,
            largerBoundXY,
            1.0,
            largerBoundZ * 2,
            largerBoundZ,
            largerBoundZ / 250);

    // Output lighting information.
    fputs("  lights: {\n"
          "    d: [\n", f);

    // Directional.
    int lightCount;
    for(lightCount = 0; lightCount < 2; lightCount++) {
        fprintf(f, "      {\n"
                   "        intensity: %f, direction: [%f, %f, %f]\n"
                   "      },\n",
                SS.lightIntensity[lightCount],
                CO(SS.lightDir[lightCount]));
    }

    // Global Ambience.
    fprintf(f, "    ],\n"
               "    a: %f\n", SS.ambientIntensity);

    for(tr = sm->l.First(); tr; tr = sm->l.NextAfter(tr)) {
        spl.IncrementTagFor(tr->a);
        spl.IncrementTagFor(tr->b);
        spl.IncrementTagFor(tr->c);
    }

    // Output all the vertices.
    SPoint *sp;
    fputs("  },\n"
          "  points: [\n", f);
    for(sp = spl.l.First(); sp; sp = spl.l.NextAfter(sp)) {
        fprintf(f, "    [%f, %f, %f],\n",
                sp->p.x / SS.exportScale,
                sp->p.y / SS.exportScale,
                sp->p.z / SS.exportScale);
    }

    fputs("  ],\n"
          "  faces: [\n", f);
    // And now all the triangular faces, in terms of those vertices.
    // This time we count from zero.
    for(tr = sm->l.First(); tr; tr = sm->l.NextAfter(tr)) {
        fprintf(f, "    [%d, %d, %d],\n",
                spl.IndexForPoint(tr->a),
                spl.IndexForPoint(tr->b),
                spl.IndexForPoint(tr->c));
    }

    // Output face normals.
    fputs("  ],\n"
          "  normals: [\n", f);
    for(tr = sm->l.First(); tr; tr = sm->l.NextAfter(tr)) {
        fprintf(f, "    [[%f, %f, %f], [%f, %f, %f], [%f, %f, %f]],\n",
                CO(tr->an), CO(tr->bn), CO(tr->cn));
    }

    fputs("  ],\n"
          "  colors: [\n", f);
    // Output triangle colors.
    for(tr = sm->l.First(); tr; tr = sm->l.NextAfter(tr)) {
        fprintf(f, "    0x%x,\n", tr->meta.color.ToARGB32());
    }

    fputs("  ],\n"
          "  edges: [\n", f);
    // Output edges. Assume user's model colors do not obscure white edges.
    for(const SOutline &so : sol->l) {
        if(so.tag == 0) continue;
        fprintf(f, "    [[%f, %f, %f], [%f, %f, %f]],\n",
                so.a.x / SS.exportScale,
                so.a.y / SS.exportScale,
                so.a.z / SS.exportScale,
                so.b.x / SS.exportScale,
                so.b.y / SS.exportScale,
                so.b.z / SS.exportScale);
    }

    fputs("  ]\n};\n", f);

    if(filename.HasExtension("html")) {
        fprintf(f, htmlend,
                basename.c_str(),
                SS.GW.scale,
                CO(SS.GW.offset),
                CO(SS.GW.projUp),
                CO(SS.GW.projRight));
    }

    spl.Clear();
}

//-----------------------------------------------------------------------------
// Export the mesh as a VRML text file / WRL.
//-----------------------------------------------------------------------------
void SolveSpaceUI::ExportMeshAsVrmlTo(FILE *f, const Platform::Path &filename, SMesh *sm) {
    struct STriangleSpan {
        STriangle *first, *past_last;

        STriangle *begin() const { return first; }
        STriangle *end() const { return past_last; }
    };


    std::string basename = filename.FileStem();
    for(auto & c : basename) {
        if(!(isalnum(c) || ((unsigned)c >= 0x80))) {
            c = '_';
        }
    }

    fprintf(f, "#VRML V2.0 utf8\n"
               "#Exported from SolveSpace %s\n"
               "\n"
               "DEF %s Transform {\n"
               "  children [",
            PACKAGE_VERSION,
            basename.c_str());


    std::map<std::uint8_t, std::vector<STriangleSpan>> opacities;
    STriangle *start          = sm->l.begin();
    std::uint8_t last_opacity = start->meta.color.alpha;
    for(auto & tr : sm->l) {
        if(tr.meta.color.alpha != last_opacity) {
            opacities[last_opacity].push_back(STriangleSpan{start, &tr});
            start = &tr;
            last_opacity = start->meta.color.alpha;
        }
    }
    opacities[last_opacity].push_back(STriangleSpan{start, sm->l.end()});

    for(auto && op : opacities) {
        fprintf(f, "\n"
                   "    Shape {\n"
                   "      appearance Appearance {\n"
                   "        material DEF %s_material_%u Material {\n"
                   "          diffuseColor %f %f %f\n"
                   "          ambientIntensity %f\n"
                   "          transparency %f\n"
                   "        }\n"
                   "      }\n"
                   "      geometry IndexedFaceSet {\n"
                   "        colorPerVertex TRUE\n"
                   "        coord Coordinate { point [\n",
                basename.c_str(),
                (unsigned)op.first,
                SS.ambientIntensity,
                SS.ambientIntensity,
                SS.ambientIntensity,
                SS.ambientIntensity,
                1.f - ((float)op.first / 255.0f));

        SPointList spl = {};

        for(const auto & sp : op.second) {
            for(const auto & tr : sp) {
                spl.IncrementTagFor(tr.a);
                spl.IncrementTagFor(tr.b);
                spl.IncrementTagFor(tr.c);
            }
        }

        // Output all the vertices.
        for(const auto &sp : spl.l) {
            fprintf(f, "          %f %f %f,\n",
                    sp.p.x / SS.exportScale,
                    sp.p.y / SS.exportScale,
                    sp.p.z / SS.exportScale);
        }

        fputs("        ] }\n"
              "        coordIndex [\n", f);
        // And now all the triangular faces, in terms of those vertices.
        for(const auto & sp : op.second) {
            for(const auto & tr : sp) {
                fprintf(f, "          %d, %d, %d, -1,\n",
                        spl.IndexForPoint(tr.a),
                        spl.IndexForPoint(tr.b),
                        spl.IndexForPoint(tr.c));
            }
        }

        fputs("        ]\n"
              "        color Color { color [\n", f);
        // Output triangle colors.
        std::vector<int> triangle_colour_ids;
        std::vector<RgbaColor> colours_present;
        for(const auto & sp : op.second) {
            for(const auto & tr : sp) {
                const auto colour_itr = std::find_if(colours_present.begin(), colours_present.end(),
                                                     [&](const RgbaColor & c) {
                                                         return c.Equals(tr.meta.color);
                                                     });
                if(colour_itr == colours_present.end()) {
                    fprintf(f, "          %.10f %.10f %.10f,\n",
                            tr.meta.color.redF(),
                            tr.meta.color.greenF(),
                            tr.meta.color.blueF());
                    triangle_colour_ids.push_back(colours_present.size());
                    colours_present.insert(colours_present.end(), tr.meta.color);
                } else {
                    triangle_colour_ids.push_back(colour_itr - colours_present.begin());
                }
            }
        }

        fputs("        ] }\n"
              "        colorIndex [\n", f);

        for(auto colour_idx : triangle_colour_ids) {
            fprintf(f, "          %d, %d, %d, -1,\n", colour_idx, colour_idx, colour_idx);
        }

        fputs("        ]\n"
              "      }\n"
              "    }\n", f);

        spl.Clear();
    }

    fputs("  ]\n"
          "}\n", f);
}

//-----------------------------------------------------------------------------
// Export a view of the model as an image; we just take a screenshot, by
// rendering the view in the usual way and then copying the pixels.
//-----------------------------------------------------------------------------
void SolveSpaceUI::ExportAsPngTo(const Platform::Path &filename) {
    screenshotFile = filename;
    // The rest of the work is done in the next redraw.
    GW.Invalidate();
}

} // namespace SolveSpace
