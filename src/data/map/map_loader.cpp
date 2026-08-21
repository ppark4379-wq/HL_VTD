#include "map_loader.hpp"

#include <cctype>
#include <fstream>
#include <stdexcept>
#include <utility>

using namespace std;

// XML의 &quot;, &amp; 같은 escape 문자를 원래 문자로 복원한다.
string decodeXmlEntities(string value) {
    const pair<const char*, const char*> entities[] = {
        {"&quot;", "\""}, {"&apos;", "'"}, {"&lt;", "<"},
        {"&gt;", ">"},   {"&amp;", "&"}};

    for (const auto& [encoded, decoded] : entities) {
        size_t position = 0;
        while ((position = value.find(encoded, position)) != string::npos) {
            value.replace(position, char_traits<char>::length(encoded), decoded);
            position += char_traits<char>::length(decoded);
        }
    }
    return value;
}

// XODR 파일의 XML 태그와 속성을 XmlElement 트리로 변환한다.
XmlElement readXmlFile(const string& path) {
    ifstream input(path, ios::binary);
    if (!input) throw runtime_error("cannot open " + path);

    string xml((istreambuf_iterator<char>(input)), istreambuf_iterator<char>());
    XmlElement document{"#document", {}, {}};
    vector<XmlElement*> stack{&document};
    size_t position = 0;

    while ((position = xml.find('<', position)) != string::npos) {
        // XML 주석은 지도 데이터가 아니므로 건너뛴다.
        if (xml.compare(position, 4, "<!--") == 0) {
            const size_t end = xml.find("-->", position + 4);
            if (end == string::npos) throw runtime_error("unterminated comment");
            position = end + 3;
            continue;
        }

        // <?xml ...?> 선언문을 건너뛴다.
        if (xml.compare(position, 2, "<?") == 0) {
            const size_t end = xml.find("?>", position + 2);
            if (end == string::npos) throw runtime_error("unterminated declaration");
            position = end + 2;
            continue;
        }

        // DOCTYPE 등 <! ...> 지시문을 건너뛴다.
        if (xml.compare(position, 2, "<!") == 0) {
            const size_t end = xml.find('>', position + 2);
            if (end == string::npos) throw runtime_error("unterminated directive");
            position = end + 1;
            continue;
        }

        const size_t end = xml.find('>', position + 1);
        if (end == string::npos) throw runtime_error("unterminated tag");
        string tag = xml.substr(position + 1, end - position - 1);
        position = end + 1;

        // </road> 같은 닫는 태그이면 부모 태그로 돌아간다.
        if (!tag.empty() && tag.front() == '/') {
            if (stack.size() > 1) stack.pop_back();
            continue;
        }

        while (!tag.empty() && isspace(static_cast<unsigned char>(tag.back())))
            tag.pop_back();
        const bool selfClosing = !tag.empty() && tag.back() == '/';
        if (selfClosing) tag.pop_back();

        // 태그 이름을 먼저 읽는다. 예: road, lane, geometry.
        size_t cursor = 0;
        while (cursor < tag.size() &&
               !isspace(static_cast<unsigned char>(tag[cursor]))) ++cursor;

        XmlElement element;
        element.name = tag.substr(0, cursor);

        // 태그 뒤의 name="value" 속성을 모두 읽는다.
        while (cursor < tag.size()) {
            while (cursor < tag.size() &&
                   isspace(static_cast<unsigned char>(tag[cursor]))) ++cursor;
            const size_t nameStart = cursor;
            while (cursor < tag.size() && tag[cursor] != '=' &&
                   !isspace(static_cast<unsigned char>(tag[cursor]))) ++cursor;
            if (cursor == nameStart) break;
            const string attributeName = tag.substr(nameStart, cursor - nameStart);

            while (cursor < tag.size() &&
                   (isspace(static_cast<unsigned char>(tag[cursor])) || tag[cursor] == '='))
                ++cursor;
            if (cursor >= tag.size() || (tag[cursor] != '"' && tag[cursor] != '\''))
                break;

            const char quote = tag[cursor++];
            const size_t valueStart = cursor;
            const size_t valueEnd = tag.find(quote, cursor);
            if (valueEnd == string::npos) break;
            element.attributes[attributeName] =
                decodeXmlEntities(tag.substr(valueStart, valueEnd - valueStart));
            cursor = valueEnd + 1;
        }

        // 읽은 태그를 현재 부모의 자식으로 저장한다.
        const string elementName = element.name;
        stack.back()->children.emplace_back(elementName, move(element));
        if (!selfClosing) stack.push_back(&stack.back()->children.back().second);
    }
    return document;
}

// XML 태그의 직계 하위 태그 중 이름이 같은 첫 태그를 찾는다.
const XmlElement* findFirstChild(const XmlElement& parent,
                                 const string& childName) {
    for (const auto& child : parent.children) {
        if (child.first == childName) return &child.second;
    }
    return nullptr;
}

// XODR 전체 XML 중 실제 지도 내용인 <OpenDRIVE> 루트만 반환한다.
XmlElement loadOpenDrive(const string& xodrPath) {
    XmlElement document = readXmlFile(xodrPath);
    const XmlElement* root = findFirstChild(document, "OpenDRIVE");

    if (!root) throw runtime_error("OpenDRIVE root not found");
    return *root;
}
