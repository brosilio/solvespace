// Calculator REPL tool window for SolveSpace.
// Text input handled via onKeyboardEvent; no modal-grab editor overlay used.

#include "solvespace.h"

namespace SolveSpace {

// Constants matching TextWindow conventions.
// CW_CHAR_W is the actual bitmap-font advance: GetWidth(codepoint)*8 = 1*8 = 8px.
// (TextWindow's CHAR_WIDTH_=9 is a wider cell that it positions explicitly;
//  DrawBitmapText uses the font's real 8px advance.)
static const int CW_CHAR_W   = 8;
static const int CW_LINE_H   = 20;
static const int CW_CHAR_H   = 16;
static const int CW_MARGIN_L = 6;
// Top padding within a row (character top = rowTop + CW_ROW_PAD).
// Matches TextWindow's +4 offset.
static const int CW_ROW_PAD  = 4;
// Height reserved at the bottom for the fixed prompt row.
static const int CW_PROMPT_H = CW_LINE_H;

// DrawBitmapText/DrawBitmapChar take y = BOTTOM of the character in screen
// coords (y=0 at top, increases downward).  DrawRect takes screen top/bottom.
// charBottom(rowTop) gives the y argument for DrawBitmapText.
// charTop(rowTop) and charBottom(rowTop) give DrawRect's t/b.
static int charTop(int rowTop)    { return rowTop + CW_ROW_PAD; }
static int charBottom(int rowTop) { return rowTop + CW_ROW_PAD + CW_CHAR_H; }

// DrawBitmapText draws space (U+0020) using an RGB pixmap, which renders as
// an opaque black rectangle. Skip spaces — just advance x — so the row
// background colour shows through.
static void DrawTextSkipSpaces(UiCanvas &ui, const std::string &str,
                                int x, int y, RgbaColor col, int zIndex) {
    for(char32_t c : ReadUTF8(str)) {
        if(c != ' ')
            ui.DrawBitmapChar(c, x, y, col, zIndex);
        x += CW_CHAR_W;
    }
}

//-----------------------------------------------------------------------------
// Colour palette
//-----------------------------------------------------------------------------

static const RgbaColor BG_WINDOW  = RGBi(  0,   0,   0);
static const RgbaColor BG_HEADER  = RGBi( 25,  25,  45);
static const RgbaColor BG_EVEN    = RGBi(  0,   0,   0);
static const RgbaColor BG_ODD     = RGBi( 16,  16,  16);
static const RgbaColor BG_INPUT   = RGBi( 20,  20,  30);
static const RgbaColor COL_SEP    = RGBi( 60,  60,  90);
static const RgbaColor COL_HDR    = RGBi(140, 140, 200);
static const RgbaColor COL_EXPR   = RGBi(140, 140, 140);
static const RgbaColor COL_RES    = RGBi(210, 210, 210);
static const RgbaColor COL_ERR    = RGBi(255, 100,  80);
static const RgbaColor COL_PROMPT = RGBi( 80, 180,  80);
static const RgbaColor COL_INPUT  = RGBi(220, 220, 220);
static const RgbaColor COL_CURSOR = RGBi(100, 200, 100);
static const RgbaColor TRANSPARENT = RgbaColor::From(0,0,0,0);

//-----------------------------------------------------------------------------
// Init / lifecycle
//-----------------------------------------------------------------------------

void CalcWindow::Init() {
    if(window) return;
    window = Platform::CreateWindow(Platform::Window::Kind::TOOL, SS.GW.window);
    if(!window) return;

    canvas = CreateRenderer();
    window->SetMinContentSize(350, 200);

    window->onClose = []() {
        SS.CW.window->SetVisible(false);
        SS.GW.showCalcWndMenuItem->SetActive(false);
    };

    window->onRender = std::bind(&CalcWindow::Paint, this);

    window->onKeyboardEvent = [this](Platform::KeyboardEvent ev) -> bool {
        if(this->KeyboardEvent(ev)) return true;
        return SS.GW.KeyboardEvent(ev);
    };

    window->onScrollbarAdjusted = [this](double pos) {
        scrollPos = std::max(0, (int)pos);
        window->Invalidate();
    };

    window->onMouseEvent = [this](Platform::MouseEvent ev) -> bool {
        if(ev.type == Platform::MouseEvent::Type::SCROLL_VERT) {
            int delta = -(int)(ev.scrollDelta * 3);
            scrollPos = std::max(0, std::min(scrollPos + delta,
                                             std::max(0, TotalRows() - VisibleRows())));
            window->SetScrollbarPosition(scrollPos);
            window->Invalidate();
            return true;
        }
        if(ev.type == Platform::MouseEvent::Type::PRESS) {
            window->Focus();
            return false;
        }
        return false;
    };

    window->SetTitle(C_("title", "Calculator"));
}

void CalcWindow::Clear() {
    history.clear();
    scrollPos = 0;
    inputBuf.clear();
    if(window) {
        window->ConfigureScrollbar(0, 0, 1);
        window->Invalidate();
    }
}

//-----------------------------------------------------------------------------
// Keyboard input
//-----------------------------------------------------------------------------

bool CalcWindow::KeyboardEvent(Platform::KeyboardEvent ev) {
    using KE = Platform::KeyboardEvent;

    if(ev.key == KE::Key::NUMLOCK) return false; // GW handles double-tap

    if(ev.type != KE::Type::PRESS) return false;

    if(ev.key == KE::Key::CHARACTER) {
        char32_t c = ev.chr;

        if(c == '\r' || c == '\n') {
            Submit(inputBuf);
            inputBuf.clear();
            window->Invalidate();
            return true;
        }
        if(c == '\x1b') {
            inputBuf.clear();
            window->Invalidate();
            return true;
        }
        if(c == '\b') {
            if(!inputBuf.empty()) inputBuf.pop_back();
            window->Invalidate();
            return true;
        }
        if(ev.controlDown) {
            if(c == 'u' || c == 'U') {
                inputBuf.clear();
                window->Invalidate();
                return true;
            }
            if(c == 'w' || c == 'W') {
                size_t end = inputBuf.find_last_not_of(' ');
                if(end != std::string::npos) {
                    size_t start = inputBuf.rfind(' ', end);
                    inputBuf.erase(start == std::string::npos ? 0 : start + 1);
                } else {
                    inputBuf.clear();
                }
                window->Invalidate();
                return true;
            }
            return false; // let Ctrl+other fall through to GW
        }
        if(c >= 32 && c < 127) {
            inputBuf += (char)c;
            window->Invalidate();
            return true;
        }
    }
    return false;
}

//-----------------------------------------------------------------------------
// Expression submission
//-----------------------------------------------------------------------------

void CalcWindow::Submit(const std::string &expr) {
    size_t s = expr.find_first_not_of(" \t");
    if(s == std::string::npos) return;
    size_t e = expr.find_last_not_of(" \t");
    std::string trimmed = expr.substr(s, e - s + 1);

    std::string raw = CalcEvaluate(trimmed);
    Entry en;
    en.expr    = trimmed;
    en.result  = raw;
    en.isError = (raw.size() >= 6 && raw.substr(0, 6) == "error:");
    history.push_back(en);

    scrollPos = std::max(0, TotalRows() - VisibleRows());
    UpdateScrollbar();
}

//-----------------------------------------------------------------------------
// Toggle
//-----------------------------------------------------------------------------

void CalcWindow::Toggle() {
    if(!window) return;
    if(!window->IsVisible()) {
        window->SetVisible(true);
        SS.GW.showCalcWndMenuItem->SetActive(true);
    }
    // Always focus: opens when hidden, brings to front and refocuses when visible.
    window->Focus();
}

//-----------------------------------------------------------------------------
// Scrollbar helpers
//-----------------------------------------------------------------------------

int CalcWindow::TotalRows() const {
    return (int)history.size() * 2;
}

int CalcWindow::VisibleRows() const {
    if(!window) return 1;
    double w, h;
    window->GetContentSize(&w, &h);
    int histH = (int)h - CW_PROMPT_H - 1;
    return std::max(1, histH / CW_LINE_H);
}

void CalcWindow::UpdateScrollbar() {
    if(!window) return;
    int total   = TotalRows();
    int visible = VisibleRows();
    scrollPos = std::max(0, std::min(scrollPos, std::max(0, total - visible)));
    window->SetScrollbarVisible(total > visible);
    window->ConfigureScrollbar(0, std::max(total, visible), visible);
    window->SetScrollbarPosition(scrollPos);
}

//-----------------------------------------------------------------------------
// Paint
//-----------------------------------------------------------------------------

void CalcWindow::Paint() {
    if(!canvas) return;

    double w, h;
    window->GetContentSize(&w, &h);

    UpdateScrollbar();

    Camera cam = {};
    cam.width      = w;
    cam.height     = h;
    cam.pixelRatio = window->GetDevicePixelRatio();
    cam.gridFit    = (window->GetDevicePixelRatio() == 1);
    cam.LoadIdentity();
    cam.offset.x   = -w / 2.0;
    cam.offset.y   = -h / 2.0;

    Lighting lt = {};
    lt.backgroundColor = BG_WINDOW;

    canvas->SetLighting(lt);
    canvas->SetCamera(cam);
    canvas->StartFrame();

    UiCanvas ui = {};
    ui.canvas = canvas;
    ui.flip   = true;

    int iw = (int)w, ih = (int)h;
    // histH: pixel height of the scrollable history area.
    // Layout: [0, histH) history  |  histH (1px sep)  |  (histH+1, ih) prompt
    int histH   = ih - CW_PROMPT_H - 1;
    int visRows = std::max(1, histH / CW_LINE_H);

    int promptTop = histH + 1; // top pixel of the prompt background area

    // ---- Pass 0 (z=0): backgrounds ------------------------------------------
    // Using explicit zIndex=0 for rects and zIndex=1 for text ensures the
    // renderer always draws backgrounds before text regardless of sort order.

    ui.DrawRect(0, iw, 0, histH, BG_EVEN, TRANSPARENT, 0);

    if(history.empty())
        ui.DrawRect(0, iw, 0, CW_LINE_H, BG_HEADER, TRANSPARENT, 0);

    for(int row = scrollPos; row < scrollPos + visRows; row++) {
        int ei = row / 2;
        if(ei < 0 || ei >= (int)history.size()) continue;
        int rowTop = (row - scrollPos) * CW_LINE_H;
        ui.DrawRect(0, iw, rowTop, rowTop + CW_LINE_H,
                    ei % 2 == 0 ? BG_EVEN : BG_ODD, TRANSPARENT, 0);
    }

    // Separator (1px) and prompt background.
    ui.DrawRect(0, iw, histH,     histH + 1, COL_SEP,  TRANSPARENT, 0);
    ui.DrawRect(0, iw, promptTop, ih,        BG_INPUT, TRANSPARENT, 0);

    // ---- Pass 1 (z=1): text -------------------------------------------------

    // DrawBitmapText y = BOTTOM of character in screen coords.
    // We use DrawTextSkipSpaces everywhere: the space glyph (U+0020) is stored
    // as an RGB pixmap and renders as an opaque black block, which covers the
    // lighter row background. Skipping it keeps the background visible.
    // Indentation is done via x-offset, not leading space characters.

    static const int INDENT = 2 * CW_CHAR_W; // 2-character visual indent

    if(history.empty()) {
        // Middle-dot (U+00B7, "\xc2\xb7") is safe — it's in the alpha texture.
        std::string hdr = "Calculator \xc2\xb7 active unit: ";
        hdr += SS.UnitName();
        DrawTextSkipSpaces(ui, hdr, CW_MARGIN_L, charBottom(0), COL_HDR, 1);
    }

    for(int row = scrollPos; row < scrollPos + visRows; row++) {
        int ei  = row / 2;
        int lin = row % 2;
        if(ei < 0 || ei >= (int)history.size()) continue;

        int rowTop = (row - scrollPos) * CW_LINE_H;
        int cy     = charBottom(rowTop);
        int tx     = CW_MARGIN_L + INDENT;
        const Entry &en = history[ei];

        if(lin == 0) {
            // Expression line — draw indented, no leading spaces.
            DrawTextSkipSpaces(ui, en.expr, tx, cy, COL_EXPR, 1);
        } else {
            if(en.isError) {
                DrawTextSkipSpaces(ui, en.result, tx, cy, COL_ERR, 1);
            } else {
                // "= result" — draw "=" then result with a small gap.
                ui.DrawBitmapChar('=', tx, cy, COL_RES, 1);
                DrawTextSkipSpaces(ui, en.result, tx + 2 * CW_CHAR_W, cy, COL_RES, 1);
            }
        }
    }

    // Prompt: ">>" label, input buffer, cursor bar.
    int promptCharY = charBottom(promptTop);
    ui.DrawBitmapChar('>', CW_MARGIN_L,              promptCharY, COL_PROMPT, 1);
    ui.DrawBitmapChar('>', CW_MARGIN_L + CW_CHAR_W,  promptCharY, COL_PROMPT, 1);

    int inputX = CW_MARGIN_L + 2 * CW_CHAR_W + 4; // small gap after ">>"
    DrawTextSkipSpaces(ui, inputBuf, inputX, promptCharY, COL_INPUT, 1);

    // Cursor: a 2px-wide vertical bar at the end of the input text.
    // DrawRect uses screen top/bottom, so use charTop/charBottom.
    int cursorX = inputX + (int)inputBuf.size() * CW_CHAR_W;
    ui.DrawRect(cursorX, cursorX + 2,
                charTop(promptTop), charBottom(promptTop),
                COL_CURSOR, TRANSPARENT, 1);

    // -------------------------------------------------------------------------
    canvas->FlushFrame();
    canvas->FinishFrame();
}

} // namespace SolveSpace
