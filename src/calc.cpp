// Expression evaluator for the SolveSpace calculator window.
// Recursive-descent parser; no external dependencies.
//
// Units: lengths are stored internally in mm, angles in radians.
// Suffixes: mm cm m in ft  (length)
//           deg rad         (angle)
// Conversion: "expr to unit" overrides the display unit for one result.

#include "solvespace.h"
#include <cmath>
#include <cctype>
#include <vector>
#include <stdexcept>

namespace SolveSpace {

//-----------------------------------------------------------------------------
// Tokenizer
//-----------------------------------------------------------------------------

enum class TT {
    NUM, IDENT,
    PLUS, MINUS, STAR, SLASH, CARET, PCT,
    LPAREN, RPAREN, COMMA,
    END
};

struct Token {
    TT          type = TT::END;
    double      num  = 0.0;
    std::string name;
};

static std::vector<Token> Tokenize(const std::string &s) {
    std::vector<Token> out;
    size_t i = 0;
    while(i < s.size()) {
        unsigned char c = (unsigned char)s[i];
        if(isspace(c)) { i++; continue; }

        if(isdigit(c) || c == '.') {
            size_t start = i;
            while(i < s.size() && isdigit((unsigned char)s[i])) i++;
            if(i < s.size() && s[i] == '.') {
                i++;
                while(i < s.size() && isdigit((unsigned char)s[i])) i++;
            }
            // Optional exponent: only consume if digits follow [eE][+-]?
            if(i < s.size() && (s[i] == 'e' || s[i] == 'E')) {
                size_t ei = i + 1;
                if(ei < s.size() && (s[ei] == '+' || s[ei] == '-')) ei++;
                if(ei < s.size() && isdigit((unsigned char)s[ei])) {
                    i = ei;
                    while(i < s.size() && isdigit((unsigned char)s[i])) i++;
                }
            }
            Token t;
            t.type = TT::NUM;
            t.num  = std::stod(s.substr(start, i - start));
            out.push_back(t);
            continue;
        }

        if(isalpha(c) || c == '_') {
            size_t start = i;
            while(i < s.size() &&
                  (isalnum((unsigned char)s[i]) || s[i] == '_')) i++;
            Token t;
            t.type = TT::IDENT;
            t.name = s.substr(start, i - start);
            out.push_back(t);
            continue;
        }

        // ' and " are shorthand for ft and in respectively.
        if(c == '\'') {
            Token t; t.type = TT::IDENT; t.name = "ft";
            out.push_back(t); i++; continue;
        }
        if(c == '"') {
            Token t; t.type = TT::IDENT; t.name = "in";
            out.push_back(t); i++; continue;
        }

        Token t;
        switch(c) {
            case '+': t.type = TT::PLUS;   break;
            case '-': t.type = TT::MINUS;  break;
            case '*': t.type = TT::STAR;   break;
            case '/': t.type = TT::SLASH;  break;
            case '^': t.type = TT::CARET;  break;
            case '%': t.type = TT::PCT;    break;
            case '(': t.type = TT::LPAREN; break;
            case ')': t.type = TT::RPAREN; break;
            case ',': t.type = TT::COMMA;  break;
            default:
                throw std::string("unexpected character '") + (char)c + "'";
        }
        out.push_back(t);
        i++;
    }
    Token end; end.type = TT::END;
    out.push_back(end);
    return out;
}

//-----------------------------------------------------------------------------
// Value type
//-----------------------------------------------------------------------------

enum class CUnit { NONE, LENGTH, ANGLE };

struct CVal {
    double v    = 0.0;
    CUnit  unit = CUnit::NONE;
    CVal() = default;
    CVal(double v, CUnit u = CUnit::NONE) : v(v), unit(u) {}
};

// For addition/subtraction: same unit → keep it; mixed or none → NONE.
static CUnit AddUnits(CUnit a, CUnit b) {
    if(a == b) return a;
    if(a == CUnit::NONE) return b;
    if(b == CUnit::NONE) return a;
    return CUnit::NONE;
}

static bool IsUnitIdent(const std::string &s) {
    return s=="mm"||s=="cm"||s=="m"||s=="in"||s=="ft"||s=="deg"||s=="rad";
}

// Apply a unit suffix to a bare number: return value in mm / radians.
static CVal ApplyUnit(double v, const std::string &u) {
    if(u == "mm")  return {v,              CUnit::LENGTH};
    if(u == "cm")  return {v * 10.0,       CUnit::LENGTH};
    if(u == "m")   return {v * 1000.0,     CUnit::LENGTH};
    if(u == "in")  return {v * 25.4,       CUnit::LENGTH};
    if(u == "ft")  return {v * 304.8,      CUnit::LENGTH};
    if(u == "deg") return {v * M_PI/180.0, CUnit::ANGLE};
    if(u == "rad") return {v,              CUnit::ANGLE};
    throw std::string("unknown unit '") + u + "'";
}

// Convert a typed value to an explicit output unit.
static CVal ConvertTo(CVal val, const std::string &u) {
    if(val.unit == CUnit::LENGTH) {
        if(u=="mm")  return {val.v,         CUnit::LENGTH};
        if(u=="cm")  return {val.v/10.0,    CUnit::LENGTH};
        if(u=="m")   return {val.v/1000.0,  CUnit::LENGTH};
        if(u=="in")  return {val.v/25.4,    CUnit::LENGTH};
        if(u=="ft")  return {val.v/304.8,   CUnit::LENGTH};
        throw std::string("cannot convert length to '") + u + "'";
    }
    if(val.unit == CUnit::ANGLE) {
        if(u=="rad") return {val.v,              CUnit::ANGLE};
        if(u=="deg") return {val.v*180.0/M_PI,   CUnit::ANGLE};
        throw std::string("cannot convert angle to '") + u + "'";
    }
    throw std::string("value is dimensionless; cannot convert to '") + u + "'";
}

//-----------------------------------------------------------------------------
// Recursive-descent parser
//-----------------------------------------------------------------------------

struct Parser {
    std::vector<Token> toks;
    size_t pos = 0;

    const Token &cur() const { return toks[pos]; }
    bool check(TT t)   const { return cur().type == t; }

    Token eat() { return toks[pos++]; }

    void expect(TT t, const char *msg) {
        if(!check(t)) throw std::string(msg);
        eat();
    }

    CVal parseExpr();
    CVal parseTerm();
    CVal parsePower();
    CVal parseUnary();
    CVal parsePostfix();
    CVal parsePrimary();
};

CVal Parser::parsePrimary() {
    if(check(TT::NUM)) {
        double v = eat().num;
        return CVal(v);
    }

    if(check(TT::LPAREN)) {
        eat();
        CVal v = parseExpr();
        expect(TT::RPAREN, "expected ')'");
        return v;
    }

    if(check(TT::IDENT)) {
        std::string name = eat().name;

        if(name == "pi")  return CVal(M_PI);
        if(name == "tau") return CVal(2.0 * M_PI);
        if(name == "e")   return CVal(M_E);

        // Function call
        if(check(TT::LPAREN)) {
            eat();
            std::vector<CVal> args;
            if(!check(TT::RPAREN)) {
                args.push_back(parseExpr());
                while(check(TT::COMMA)) { eat(); args.push_back(parseExpr()); }
            }
            expect(TT::RPAREN, "expected ')' after function arguments");

            // Helper: get n-th argument's raw value (strips unit).
            auto arg = [&](size_t n, const char *fn) {
                if(args.size() <= n)
                    throw std::string(fn) + " requires " +
                          std::to_string(n + 1) + " argument(s)";
                return args[n].v;
            };

            if(name == "sqrt")  return CVal(sqrt(arg(0,"sqrt")));
            if(name == "cbrt")  return CVal(cbrt(arg(0,"cbrt")));
            if(name == "abs")   return CVal(fabs(arg(0,"abs")));
            if(name == "floor") return CVal(floor(arg(0,"floor")));
            if(name == "ceil")  return CVal(ceil(arg(0,"ceil")));
            if(name == "round") return CVal(round(arg(0,"round")));
            if(name == "sign")  return CVal(arg(0,"sign") >= 0 ? 1.0 : -1.0);
            if(name == "exp")   return CVal(exp(arg(0,"exp")));
            if(name == "log" || name == "ln") return CVal(log(arg(0,name.c_str())));
            if(name == "log10") return CVal(log10(arg(0,"log10")));
            if(name == "log2")  return CVal(log2(arg(0,"log2")));
            if(name == "sin")   return CVal(sin(arg(0,"sin")));
            if(name == "cos")   return CVal(cos(arg(0,"cos")));
            if(name == "tan")   return CVal(tan(arg(0,"tan")));
            if(name == "asin")  return CVal(asin(arg(0,"asin")),  CUnit::ANGLE);
            if(name == "acos")  return CVal(acos(arg(0,"acos")),  CUnit::ANGLE);
            if(name == "atan")  return CVal(atan(arg(0,"atan")),  CUnit::ANGLE);
            if(name == "atan2") return CVal(atan2(arg(0,"atan2"), arg(1,"atan2")), CUnit::ANGLE);
            if(name == "pow")   return CVal(pow(arg(0,"pow"), arg(1,"pow")));
            if(name == "hypot") return CVal(hypot(arg(0,"hypot"), arg(1,"hypot")));
            if(name == "min")   return CVal(std::min(arg(0,"min"), arg(1,"min")));
            if(name == "max")   return CVal(std::max(arg(0,"max"), arg(1,"max")));

            throw std::string("unknown function '") + name + "'";
        }

        throw std::string("unknown identifier '") + name + "'";
    }

    throw std::string("expected a number or expression");
}

CVal Parser::parsePostfix() {
    CVal v = parsePrimary();
    // Optional unit suffix immediately after a primary.
    if(check(TT::IDENT) && IsUnitIdent(cur().name)) {
        if(v.unit != CUnit::NONE)
            throw std::string("value already has a unit");
        std::string u = eat().name;
        v = ApplyUnit(v.v, u);

        // Compound feet+inches: consume an immediately following inches value.
        // Handles 1ft6in, 1'6", 1'6, 5'11", etc.
        // The ' → ft and " → in mappings are done in the tokenizer.
        if(u == "ft" && check(TT::NUM)) {
            double inchPart = eat().num;
            // Consume optional explicit inch marker.
            if(check(TT::IDENT) && cur().name == "in") eat();
            v.v += inchPart * 25.4;
        }
    }
    return v;
}

CVal Parser::parseUnary() {
    if(check(TT::MINUS)) { eat(); CVal v = parseUnary(); v.v = -v.v; return v; }
    if(check(TT::PLUS))  { eat(); return parseUnary(); }
    return parsePostfix();
}

CVal Parser::parsePower() {
    CVal base = parseUnary();
    if(check(TT::CARET)) {
        eat();
        CVal exp = parseUnary();
        return CVal(pow(base.v, exp.v));
    }
    return base;
}

CVal Parser::parseTerm() {
    CVal v = parsePower();
    while(check(TT::STAR) || check(TT::SLASH) || check(TT::PCT)) {
        TT op = eat().type;
        CVal r = parsePower();
        if(op == TT::STAR) {
            // NONE*X→X, X*NONE→X, X*X→drop unit (area unsupported)
            CUnit u = (v.unit == CUnit::NONE) ? r.unit :
                      (r.unit == CUnit::NONE) ? v.unit : CUnit::NONE;
            v = CVal(v.v * r.v, u);
        } else if(op == TT::SLASH) {
            if(r.v == 0.0) throw std::string("division by zero");
            // X/X→NONE (ratio), X/NONE→X, NONE/X→NONE
            CUnit u = (v.unit == r.unit && v.unit != CUnit::NONE) ? CUnit::NONE :
                      (r.unit == CUnit::NONE) ? v.unit : CUnit::NONE;
            v = CVal(v.v / r.v, u);
        } else {
            if(r.v == 0.0) throw std::string("modulo by zero");
            v = CVal(fmod(v.v, r.v));
        }
    }
    return v;
}

CVal Parser::parseExpr() {
    CVal v = parseTerm();
    while(check(TT::PLUS) || check(TT::MINUS)) {
        TT op = eat().type;
        CVal r = parseTerm();
        CUnit u = AddUnits(v.unit, r.unit);
        v = CVal(op == TT::PLUS ? v.v + r.v : v.v - r.v, u);
    }
    return v;
}

//-----------------------------------------------------------------------------
// Formatting and public entry point
//-----------------------------------------------------------------------------

static std::string FmtNum(double v) {
    if(v == 0.0) return "0";
    char buf[64];
    snprintf(buf, sizeof(buf), "%.10g", v);
    return buf;
}

std::string CalcEvaluate(const std::string &input) {
    if(input.empty()) return "";
    try {
        Parser p;
        p.toks = Tokenize(input);

        CVal result = p.parseExpr();

        // Optional top-level "to unit" conversion.
        std::string forceUnit;
        if(p.check(TT::IDENT) && p.cur().name == "to") {
            p.eat();
            if(!p.check(TT::IDENT) || !IsUnitIdent(p.cur().name))
                throw std::string("expected a unit after 'to'");
            forceUnit = p.eat().name;
            result = ConvertTo(result, forceUnit);
        }

        if(!p.check(TT::END))
            throw std::string("unexpected input after expression");

        // Format according to unit type.
        // No spaces in output — spaces paint opaque black blocks over the
        // row background due to the bitmap font's space glyph using RGB format.
        if(!forceUnit.empty()) {
            return FmtNum(result.v) + forceUnit;
        }
        switch(result.unit) {
            case CUnit::NONE:
                return FmtNum(result.v);
            case CUnit::LENGTH: {
                double inUnit = result.v / SS.MmPerUnit();
                return FmtNum(inUnit) + SS.UnitName();
            }
            case CUnit::ANGLE: {
                double deg = result.v * 180.0 / M_PI;
                return FmtNum(deg) + "\xc2\xb0";
            }
        }
        return FmtNum(result.v);

    } catch(const std::string &e) {
        return "error:" + e;
    } catch(const std::exception &ex) {
        return std::string("error:") + ex.what();
    }
}

} // namespace SolveSpace
