// ---------------------------------------------------------------------------
// meshexpander_cli — command-line tool for multi-part assembly expansion
//
// Usage:
//   meshexpander_cli --input assembly.stp --output result.obj --d 0.002
//
// Requires: MESHEXPANDER_BUILD_IO=ON  (Assimp-based I/O)
// ---------------------------------------------------------------------------

#include <iostream>
#include <string>
#include <stdexcept>

#include "expander/AssemblyExpander.hpp"
#include "io/AssimpLoader.hpp"
#include "io/AssimpExporter.hpp"

using namespace expander;
using namespace expander::io;

namespace {

struct Args {
    std::string input;
    std::string output;
    double      d              = 0.0;
    int         resolution     = 64;
    bool        mergeContained = true;
    bool        verbose        = false;
};

void printUsage(const char* prog) {
    std::cerr
        << "Usage: " << prog
        << " --input <file> --output <file> --d <dist> [options]\n"
        << "\n"
        << "Required:\n"
        << "  --input  <path>   Input model (any Assimp-supported format)\n"
        << "  --output <path>   Output file (format from extension: obj stl fbx gltf ...)\n"
        << "  --d <distance>    Expansion distance in model units\n"
        << "\n"
        << "Optional:\n"
        << "  --resolution <n>  Voxel resolution for concave parts [default: 64]\n"
        << "  --no-merge        Skip bounding-box containment merging\n"
        << "  --verbose         Print per-step statistics\n"
        << "  --list-formats    Print supported output formats and exit\n";
}

Args parseArgs(int argc, char** argv) {
    Args args;
    bool hasInput = false, hasOutput = false, hasD = false;

    for (int i = 1; i < argc; ++i) {
        std::string a = argv[i];
        if (a == "--list-formats") {
            std::cout << "Supported output formats:\n";
            for (const auto& fmt : AssimpExporter::listFormats())
                std::cout << "  ." << fmt << "\n";
            std::exit(0);
        } else if (a == "--input"      && i+1 < argc) { args.input  = argv[++i]; hasInput  = true; }
        else if  (a == "--output"     && i+1 < argc) { args.output = argv[++i]; hasOutput = true; }
        else if  (a == "--d"          && i+1 < argc) { args.d = std::stod(argv[++i]); hasD = true; }
        else if  (a == "--resolution" && i+1 < argc) { args.resolution = std::stoi(argv[++i]); }
        else if  (a == "--no-merge")                 { args.mergeContained = false; }
        else if  (a == "--verbose")                  { args.verbose = true; }
        else { throw std::runtime_error("unknown argument: " + a); }
    }

    if (!hasInput || !hasOutput || !hasD)
        throw std::runtime_error("--input, --output, and --d are required");
    if (args.d <= 0.0)
        throw std::runtime_error("--d must be positive");

    return args;
}

} // namespace

int main(int argc, char** argv) {
    Args args;
    try {
        args = parseArgs(argc, argv);
    } catch (const std::exception& e) {
        std::cerr << "Argument error: " << e.what() << "\n\n";
        printUsage(argv[0]);
        return 1;
    }

    try {
        // ── Load ──────────────────────────────────────────────────────────
        AssimpLoader loader;
        auto parts = loader.load(args.input);
        if (parts.empty())
            throw std::runtime_error("no mesh parts found in: " + args.input);
        if (args.verbose)
            std::cout << "Loaded " << parts.size() << " part(s) from " << args.input << "\n";

        // ── Merge contained parts ─────────────────────────────────────────
        if (args.mergeContained) {
            const auto before = parts.size();
            parts = AssemblyExpander::mergeContained(parts);
            if (args.verbose && parts.size() < before)
                std::cout << "Merged: " << before << " -> " << parts.size()
                          << " part(s) (bounding-box containment)\n";
        }

        // ── Expand ────────────────────────────────────────────────────────
        AssemblyExpander::Options opts;
        opts.resolution = args.resolution;
        AssemblyExpander expander(opts);

        if (args.verbose) {
            int nConvex = 0;
            for (const auto& p : parts)
                if (AssemblyExpander::isConvex(p)) ++nConvex;
            std::cout << "Parts: " << nConvex << " convex, "
                      << (static_cast<int>(parts.size()) - nConvex) << " concave\n";
        }

        Mesh result = expander.expandMerged(parts, args.d);
        if (args.verbose)
            std::cout << "Result: " << result.numVertices() << " vertices, "
                      << result.numFaces() << " faces\n";

        // ── Export ────────────────────────────────────────────────────────
        AssimpExporter exporter;
        exporter.write(args.output, {result});
        std::cout << "Written: " << args.output << "\n";

        return 0;

    } catch (const std::exception& e) {
        std::cerr << "Error: " << e.what() << "\n";
        return 2;
    }
}
