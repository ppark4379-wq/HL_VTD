#include "map_loader.hpp"

#include <cctype>
#include <fstream>
#include <stdexcept>
#include <utility>


// XML??&quot;, &amp; 媛숈? escape 臾몄옄瑜??먮옒 臾몄옄濡?蹂듭썝?쒕떎.
std::string decodeXmlEntities(std::string value) {
    const std::pair<const char*, const char*> entities[] = {
        {"&quot;", "\""}, {"&apos;", "'"}, {"&lt;", "<"},
        {"&gt;", ">"},   {"&amp;", "&"}};

    for (const auto& [encoded, decoded] : entities) {
        std::size_t position = 0;
        while ((position = value.find(encoded, position)) != std::string::npos) {
            value.replace(position, std::char_traits<char>::length(encoded), decoded);
            position += std::char_traits<char>::length(decoded);
        }
    }
    return value;
}

// XODR ?뚯씪??XML ?쒓렇? ?띿꽦??XmlElement ?몃━濡?蹂?섑븳??
XmlElement readXmlFile(const std::string& path) {
    std::ifstream input(path, std::ios::binary);
    if (!input) throw std::runtime_error("cannot open " + path);

    std::string xml((std::istreambuf_iterator<char>(input)), std::istreambuf_iterator<char>());
    XmlElement document{"#document", {}, {}};
    std::vector<XmlElement*> stack{&document};
    std::size_t position = 0;

    while ((position = xml.find('<', position)) != std::string::npos) {
        // XML 二쇱꽍? 吏???곗씠?곌? ?꾨땲誘濡?嫄대꼫?대떎.
        if (xml.compare(position, 4, "<!--") == 0) {
            const std::size_t end = xml.find("-->", position + 4);
            if (end == std::string::npos) throw std::runtime_error("unterminated comment");
            position = end + 3;
            continue;
        }

        // <?xml ...?> ?좎뼵臾몄쓣 嫄대꼫?대떎.
        if (xml.compare(position, 2, "<?") == 0) {
            const std::size_t end = xml.find("?>", position + 2);
            if (end == std::string::npos) throw std::runtime_error("unterminated declaration");
            position = end + 2;
            continue;
        }

        // DOCTYPE ??<! ...> 吏?쒕Ц??嫄대꼫?대떎.
        if (xml.compare(position, 2, "<!") == 0) {
            const std::size_t end = xml.find('>', position + 2);
            if (end == std::string::npos) throw std::runtime_error("unterminated directive");
            position = end + 1;
            continue;
        }

        const std::size_t end = xml.find('>', position + 1);
        if (end == std::string::npos) throw std::runtime_error("unterminated tag");
        std::string tag = xml.substr(position + 1, end - position - 1);
        position = end + 1;

        // </road> 媛숈? ?ル뒗 ?쒓렇?대㈃ 遺紐??쒓렇濡??뚯븘媛꾨떎.
        if (!tag.empty() && tag.front() == '/') {
            if (stack.size() > 1) stack.pop_back();
            continue;
        }

        while (!tag.empty() && std::isspace(static_cast<unsigned char>(tag.back())))
            tag.pop_back();
        const bool selfClosing = !tag.empty() && tag.back() == '/';
        if (selfClosing) tag.pop_back();

        // ?쒓렇 ?대쫫??癒쇱? ?쎈뒗?? ?? road, lane, geometry.
        std::size_t cursor = 0;
        while (cursor < tag.size() &&
               !std::isspace(static_cast<unsigned char>(tag[cursor]))) ++cursor;

        XmlElement element;
        element.name = tag.substr(0, cursor);

        // ?쒓렇 ?ㅼ쓽 name="value" ?띿꽦??紐⑤몢 ?쎈뒗??
        while (cursor < tag.size()) {
            while (cursor < tag.size() &&
                   std::isspace(static_cast<unsigned char>(tag[cursor]))) ++cursor;
            const std::size_t nameStart = cursor;
            while (cursor < tag.size() && tag[cursor] != '=' &&
                   !std::isspace(static_cast<unsigned char>(tag[cursor]))) ++cursor;
            if (cursor == nameStart) break;
            const std::string attributeName = tag.substr(nameStart, cursor - nameStart);

            while (cursor < tag.size() &&
                   (std::isspace(static_cast<unsigned char>(tag[cursor])) || tag[cursor] == '='))
                ++cursor;
            if (cursor >= tag.size() || (tag[cursor] != '"' && tag[cursor] != '\''))
                break;

            const char quote = tag[cursor++];
            const std::size_t valueStart = cursor;
            const std::size_t valueEnd = tag.find(quote, cursor);
            if (valueEnd == std::string::npos) break;
            element.attributes[attributeName] =
                decodeXmlEntities(tag.substr(valueStart, valueEnd - valueStart));
            cursor = valueEnd + 1;
        }

        // ?쎌? ?쒓렇瑜??꾩옱 遺紐⑥쓽 ?먯떇?쇰줈 ??ν븳??
        const std::string elementName = element.name;
        stack.back()->children.emplace_back(elementName, std::move(element));
        if (!selfClosing) stack.push_back(&stack.back()->children.back().second);
    }
    return document;
}

// XML ?쒓렇??吏곴퀎 ?섏쐞 ?쒓렇 以??대쫫??媛숈? 泥??쒓렇瑜?李얜뒗??
const XmlElement* findFirstChild(const XmlElement& parent,
                                 const std::string& childName) {
    for (const auto& child : parent.children) {
        if (child.first == childName) return &child.second;
    }
    return nullptr;
}

// XODR ?꾩껜 XML 以??ㅼ젣 吏???댁슜??<OpenDRIVE> 猷⑦듃留?諛섑솚?쒕떎.
XmlElement loadOpenDrive(const std::string& xodrPath) {
    XmlElement document = readXmlFile(xodrPath);
    const XmlElement* root = findFirstChild(document, "OpenDRIVE");

    if (!root) throw std::runtime_error("OpenDRIVE root not found");
    return *root;
}
