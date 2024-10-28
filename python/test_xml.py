import xml.etree.ElementTree as et 


root = et.Element('testing')

ver = et.SubElement(root,'Version',hello='oi')
ver.text = '5.1'

# Save
tree = et.ElementTree(root)
tree.write('test.xml', encoding='utf-8', xml_declaration=True)