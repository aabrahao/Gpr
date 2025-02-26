import xml.etree.ElementTree as et 

version_major = 1
version_minor = 4


sdf = et.Element('sdf', version=f'{version_major}.{version_minor}')

model = et.SubElement(sdf, 'model', name='my_model')

pose = et.SubElement(model, 'pose')
pose.text = f'{0} {0} {0.5} {0} {0} {0}'

static = et.SubElement(model, 'static')
static.text = 'true'

link = et.SubElement(model, 'link', name='link')

inertial = et.SubElement(link, 'inertial')

mass = et.SubElement(inertial, 'mass')
mass.text = '1.0'


# Save
tree = et.ElementTree(sdf)
tree.write('test.sdf', encoding='utf-8', xml_declaration=True)