

from kivy.lang import Builder
from kivymd.app import MDApp
from kivymd.uix.menu import MDDropdownMenu


class DOT_PAQUITOP_GUI(MDApp):

    def __init__(self, **kwargs):
        super().__init__(**kwargs)

        self.screen = Builder.load_file('dot_first_GUI.kv')

        places_items = [
            {
                "text": f"Letto 1",
                "viewclass": "OneLineListItem",
                "on_press": lambda place=f"Letto 1": self.screen.ids.drop_item.set_item(place),
            },
            {
                "text": f"Letto 2",
                "viewclass": "OneLineListItem",
                "on_press": lambda place=f"Letto 2": self.screen.ids.drop_item.set_item(place),
            },
            {
                "text": f"Sala Prelievi",
                "viewclass": "OneLineListItem",
                "on_press": lambda place=f"Sala Prelievi": self.screen.ids.drop_item.set_item(place),
            },
            {
                "text": f"Laboratorio",
                "viewclass": "OneLineListItem",
                "on_press": lambda place=f"Laboratorio": self.screen.ids.drop_item.set_item(place),
            }
        ]

        self.drop_item = MDDropdownMenu(
            caller=self.screen.ids.drop_item,
            items=places_items,
            width_mult=4,
        )


    def build(self):
        return self.screen

    def startPAQUITOP(self,*args):
        print("Avvio PAQUITOP")
        self.screen.ids.statusFB._set_text("DEMO avviata")

    def stopPAQUITOP(self,*args):
        print("Fermo PAQUITOP")
        self.screen.ids.statusFB._set_text("DEMO fermata")

    def go2(self,goal):
        print("Naviga fino a " + goal)
        self.screen.ids.statusFB._set_text("Navigo verso " + goal)

    def VSMeasure(self,*args):
        print("Misura de parametri vitali")
        self.screen.ids.statusFB._set_text("Parametri vitali acquisiti")

    def TabletLift(self,*args):
        print("Alzo il tablet")
        self.screen.ids.statusFB._set_text("Alzo il tablet")

    def TabletStore(self,*args):
        print("Ripongo il tablet")
        self.screen.ids.statusFB._set_text("Ripongo il tablet")





if __name__ == '__main__':

    DOT_PAQUITOP_GUI().run()

